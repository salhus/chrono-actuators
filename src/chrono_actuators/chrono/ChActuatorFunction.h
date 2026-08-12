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
// Chrono binding: ChFunction adapter for ChLinkMotorLinearForce /
// ChLinkMotorRotationTorque via time-dependent motor functions.
// =============================================================================

#ifndef CHRONO_ACTUATORS_CHRONO_FUNCTION_H
#define CHRONO_ACTUATORS_CHRONO_FUNCTION_H

#include <memory>

#include "chrono/core/ChTypes.h"
#include "chrono/functions/ChFunctionBase.h"
#include "chrono_actuators/ChApiActuators.h"
#include "chrono_actuators/models/ActuatorModel.h"
#include "chrono_actuators/models/Telemetry.h"

namespace chrono {
namespace actuators {

/// ChFunction adapter for motor-link actuator models.
///
/// Binds a zero-state ActuatorModel as the motor function for
/// ChLinkMotorLinearForce or ChLinkMotorRotationTorque.  The function
/// value at time t is the effort returned by the model.
///
/// The kinematic state (displacement, velocity) must be supplied externally
/// and frozen before Chrono calls GetVal(); use UpdateState() once per step.
class ChApiActuators ChActuatorFunction : public chrono::ChFunction {
  public:
    ChActuatorFunction(std::shared_ptr<ActuatorModel> model,
                       const ActuatorCommand&         command)
        : model_(std::move(model)), command_(command) {}

    void FreezeCommand(const ActuatorCommand& cmd) { command_ = cmd; }
    void UpdateState(const ActuatorState& state) { state_ = state; }
    const ActuatorTelemetry& GetTelemetry() const { return telemetry_; }

    double GetVal(double x) const override {
        ActuatorState s = state_;
        s.time = x;
        const double effort = model_->ComputeEffort(command_, s);
        telemetry_ = ActuatorTelemetry{};
        telemetry_.effort           = effort;
        telemetry_.mechanical_power = effort * s.velocity;
        return effort;
    }

    ChFunction* Clone() const override {
        return new ChActuatorFunction(*this);
    }

  private:
    std::shared_ptr<ActuatorModel> model_;
    ActuatorCommand                command_;
    ActuatorState                  state_;
    mutable ActuatorTelemetry      telemetry_;
};

}  // namespace actuators
}  // namespace chrono

#endif  // CHRONO_ACTUATORS_CHRONO_FUNCTION_H
