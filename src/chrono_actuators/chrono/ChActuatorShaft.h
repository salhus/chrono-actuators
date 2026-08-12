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
#include <stdexcept>

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
/// Supports zero-state ActuatorModels only (GetNumStates() == 0).
/// Stateful stiff models (e.g. ElectricActuatorModel) must be integrated
/// monolithically with the Chrono system via ChActuatorDynamics.
/// A runtime check enforces this invariant on construction.
///
/// Sign convention: positive effort drives the output shaft in the positive
/// angular direction.
class ChApiActuators ChActuatorShaft {
  public:
    /// @param model   Zero-state ActuatorModel (GetNumStates() == 0 required).
    /// @param motor   Shaft motor link to drive.
    /// @throws std::invalid_argument if model->GetNumStates() > 0.
    ChActuatorShaft(std::shared_ptr<ActuatorModel>      model,
                    std::shared_ptr<ChShaftsMotorTorque> motor,
                    const EnvelopeParams&                envelope = EnvelopeParams{})
        : model_(std::move(model))
        , motor_(std::move(motor))
        , envelope_(envelope)
        , prev_effort_(0.0) {
        if (model_ && model_->GetNumStates() > 0)
            throw std::invalid_argument(
                "ChActuatorShaft: stateful models (GetNumStates() > 0) must use "
                "ChActuatorDynamics for monolithic integration with the Chrono system. "
                "Use a zero-state model or replace with ChActuatorDynamics.");
    }

    /// Call once per accepted step.
    /// @param command   Frozen command for this step.
    /// @param sim_time  Current simulation time [s].
    /// @param dt        Step size [s].
    void Advance(const ActuatorCommand& command, double sim_time, double dt) {
        ActuatorState state;
        state.displacement = motor_->GetMotorRot();
        state.velocity     = motor_->GetMotorRot_dt();
        state.time         = sim_time;

        double effort = model_->ComputeEffort(command, state);

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
    ActuatorTelemetry                    telemetry_;
};

}  // namespace actuators
}  // namespace chrono

#endif  // CHRONO_ACTUATORS_CHRONO_SHAFT_H
