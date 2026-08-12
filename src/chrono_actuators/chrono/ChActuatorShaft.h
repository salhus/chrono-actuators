// =============================================================================
// PROJECT CHRONO - http://projectchrono.org
//
// Copyright (c) 2024 projectchrono.org
// All rights reserved.
//
// Use of this source code is governed by a BSD-style license that can be found
// in the LICENSE file at the top level of the distribution and at
// http://projectchrono.org/license-chrono.txt.
//
// =============================================================================
// Authors: chrono-actuators contributors
// =============================================================================
//
// Chrono binding: ChShaft / ChShaftsMotor adapter.
// =============================================================================

#ifndef CHRONO_ACTUATORS_CHRONO_SHAFT_H
#define CHRONO_ACTUATORS_CHRONO_SHAFT_H

#include <memory>
#include <vector>

#include "chrono/core/ChTypes.h"
#include "chrono/functions/ChFunctionConst.h"
#include "chrono/physics/ChShaft.h"
#include "chrono/physics/ChShaftsMotorTorque.h"
#include "chrono_actuators/ChApiActuators.h"
#include "chrono_actuators/models/ActuatorModel.h"
#include "chrono_actuators/models/Envelope.h"
#include "chrono_actuators/models/Telemetry.h"

namespace chrono {
namespace actuators {

/// 1-D shaft motor adapter.
///
/// Wraps an ActuatorModel and applies its output torque to a ChShaftsMotorTorque
/// link once per step via Advance().  Use in driveline / powertrain topologies.
///
/// Sign convention: positive effort drives the output shaft in the positive
/// angular direction.
class ChApiActuators ChActuatorShaft {
  public:
    /// @param model   ActuatorModel (zero-state or stateful).
    /// @param motor   Shaft motor link to drive.
    ChActuatorShaft(std::shared_ptr<ActuatorModel>      model,
                    std::shared_ptr<ChShaftsMotorTorque> motor,
                    const EnvelopeParams&                envelope = EnvelopeParams{})
        : model_(std::move(model))
        , motor_(std::move(motor))
        , envelope_(envelope)
        , prev_effort_(0.0)
        , states_(model_ ? model_->GetNumStates() : 0, 0.0)
        , states_initialized_(false) {}

    /// Call once per accepted step.
    /// @param command   Frozen command for this step.
    /// @param sim_time  Current simulation time [s].
    /// @param dt        Step size [s].
    void Advance(const ActuatorCommand& command, double sim_time, double dt) {
        ActuatorState state;
        state.displacement = motor_->GetMotorRot();
        state.velocity     = motor_->GetMotorRot_dt();
        state.time         = sim_time;

        if (!states_initialized_ && !states_.empty()) {
            model_->SetInitialConditions(states_.data());
            states_initialized_ = true;
        }

        double effort = 0.0;
        if (states_.empty()) {
            effort = model_->ComputeEffort(command, state);
        } else {
            std::vector<double> rhs(states_.size(), 0.0);
            model_->CalculateRHS(sim_time, states_.data(), state, command, rhs.data());
            for (std::size_t i = 0; i < states_.size(); ++i)
                states_[i] += dt * rhs[i];
            effort = model_->EffortFromStates(states_.data(), state);
        }

        telemetry_ = ActuatorTelemetry{};
        effort = ApplyPureEnvelope(effort, state.velocity, envelope_, telemetry_);
        effort = ApplyRateLimit(effort, prev_effort_, dt, envelope_.max_rate, telemetry_);

        motor_->SetTorqueFunction(chrono_types::make_shared<ChFunctionConst>(effort));

        telemetry_.effort           = effort;
        telemetry_.mechanical_power = effort * state.velocity;
    }

    const ActuatorTelemetry& GetTelemetry() const { return telemetry_; }

  private:
    std::shared_ptr<ActuatorModel>       model_;
    std::shared_ptr<ChShaftsMotorTorque> motor_;
    EnvelopeParams                       envelope_;
    double                               prev_effort_;
    std::vector<double>                  states_;
    bool                                 states_initialized_;
    ActuatorTelemetry                    telemetry_;
};

}  // namespace actuators
}  // namespace chrono

#endif  // CHRONO_ACTUATORS_CHRONO_SHAFT_H
