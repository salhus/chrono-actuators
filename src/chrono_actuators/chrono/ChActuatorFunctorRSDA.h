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
// Chrono binding: RSDA torque functor adapter.
// =============================================================================

#ifndef CHRONO_ACTUATORS_CHRONO_FUNCTOR_RSDA_H
#define CHRONO_ACTUATORS_CHRONO_FUNCTOR_RSDA_H

#include <memory>

#include "chrono/physics/ChLinkRSDA.h"
#include "chrono_actuators/ChApiActuators.h"
#include "chrono_actuators/models/ActuatorModel.h"
#include "chrono_actuators/models/Envelope.h"
#include "chrono_actuators/models/Telemetry.h"

namespace chrono {
namespace actuators {

/// RSDA torque functor binding for zero-state actuator models.
///
/// Sign convention: positive torque acts in the positive-angle direction,
/// consistent with Chrono's RSDA torque sign convention.
class ChApiActuators ChActuatorFunctorRSDA : public chrono::ChLinkRSDA::TorqueFunctor {
  public:
    ChActuatorFunctorRSDA(std::shared_ptr<ActuatorModel> model,
                          const ActuatorCommand&         command,
                          const EnvelopeParams&          envelope = EnvelopeParams{})
        : model_(std::move(model)), command_(command), envelope_(envelope) {}

    void FreezeCommand(const ActuatorCommand& command) { command_ = command; }
    const ActuatorTelemetry& GetTelemetry() const { return telemetry_; }

    double evaluate(double time,
                    double rest_angle,
                    double angle,
                    double vel,
                    const chrono::ChLinkRSDA& /*link*/) override {
        ActuatorState state;
        state.displacement = angle - rest_angle;
        state.velocity     = vel;
        state.time         = time;

        double effort = model_->ComputeEffort(command_, state);

        telemetry_ = ActuatorTelemetry{};
        effort = ApplyPureEnvelope(effort, vel, envelope_, telemetry_);
        telemetry_.effort           = effort;
        telemetry_.mechanical_power = effort * vel;

        return effort;
    }

  private:
    std::shared_ptr<ActuatorModel> model_;
    ActuatorCommand                command_;
    EnvelopeParams                 envelope_;
    mutable ActuatorTelemetry      telemetry_;
};

}  // namespace actuators
}  // namespace chrono

#endif  // CHRONO_ACTUATORS_CHRONO_FUNCTOR_RSDA_H
