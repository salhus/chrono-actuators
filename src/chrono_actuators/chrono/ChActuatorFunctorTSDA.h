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
// Chrono binding: TSDA force functor adapter.
// =============================================================================

#ifndef CHRONO_ACTUATORS_CHRONO_FUNCTOR_TSDA_H
#define CHRONO_ACTUATORS_CHRONO_FUNCTOR_TSDA_H

#include <memory>

#include "chrono/physics/ChLinkTSDA.h"
#include "chrono_actuators/ChApiActuators.h"
#include "chrono_actuators/models/ActuatorModel.h"
#include "chrono_actuators/models/Envelope.h"
#include "chrono_actuators/models/Telemetry.h"

namespace chrono {
namespace actuators {

/// TSDA force functor binding for zero-state actuator models.
///
/// Attaches an ActuatorModel to a ChLinkTSDA spring-damper-actuator link.
/// The model receives kinematic state from the TSDA geometry and returns
/// effort that replaces the TSDA spring/damper force.
///
/// Sign convention: positive effort acts to extend (increase TSDA length),
/// consistent with Chrono's TSDA force sign convention.
///
/// Invariant A is enforced by ComputeEffort being declared const on the model.
/// The functor evaluate() is non-const to match the ChLinkTSDA::ForceFunctor
/// base class signature, but it performs no mutation; all mutable state is
/// limited to the telemetry cache.
class ChApiActuators ChActuatorFunctorTSDA : public chrono::ChLinkTSDA::ForceFunctor {
  public:
    /// @param model    Zero-state ActuatorModel (GetNumStates() must be 0).
    /// @param command  Frozen command for this step (caller freezes before functor evaluation).
    /// @param envelope Pure envelope parameters (saturation, deadband, efficiency).
    ChActuatorFunctorTSDA(std::shared_ptr<ActuatorModel> model,
                          const ActuatorCommand&         command,
                          const EnvelopeParams&          envelope = EnvelopeParams{})
        : model_(std::move(model)), command_(command), envelope_(envelope) {}

    /// Update the frozen command for the current step.
    /// Call once per accepted step, NOT inside Chrono's force evaluation loop.
    void FreezeCommand(const ActuatorCommand& command) { command_ = command; }

    /// Access last-step telemetry.
    const ActuatorTelemetry& GetTelemetry() const { return telemetry_; }

    double evaluate(double time,
                    double rest_length,
                    double length,
                    double vel,
                    const chrono::ChLinkTSDA& /*link*/) override {
        ActuatorState state;
        state.displacement = length - rest_length;
        state.velocity     = vel;
        state.time         = time;

        double effort = model_->ComputeEffort(command_, state);

        telemetry_ = ActuatorTelemetry{};
        telemetry_.effort = effort;
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

#endif  // CHRONO_ACTUATORS_CHRONO_FUNCTOR_TSDA_H
