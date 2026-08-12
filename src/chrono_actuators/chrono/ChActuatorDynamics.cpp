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

#include "chrono_actuators/chrono/ChActuatorDynamics.h"

#include <stdexcept>

namespace chrono {
namespace actuators {

ChActuatorDynamics::ChActuatorDynamics(std::shared_ptr<ActuatorModel> model,
                                       std::shared_ptr<ChBody>        body1,
                                       std::shared_ptr<ChBody>        body2,
                                       bool                           local,
                                       const ChVector3d&              loc1,
                                       const ChVector3d&              loc2,
                                       const EnvelopeParams&          envelope)
    : model_(std::move(model))
    , envelope_(envelope)
    , body1_(std::move(body1))
    , body2_(std::move(body2))
    , local_(local)
    , loc1_(loc1)
    , loc2_(loc2)
    , attached_(true) {
    if (!model_)
        throw std::invalid_argument("ChActuatorDynamics: model must not be null");
    if (model_->GetNumStates() == 0)
        throw std::invalid_argument("ChActuatorDynamics: stateful model required");
    ChExternalDynamicsODE::Initialize();
}

ChActuatorDynamics::ChActuatorDynamics(std::shared_ptr<ActuatorModel> model,
                                       const EnvelopeParams&          envelope)
    : model_(std::move(model)), envelope_(envelope), attached_(false) {
    if (!model_)
        throw std::invalid_argument("ChActuatorDynamics: model must not be null");
    if (model_->GetNumStates() == 0)
        throw std::invalid_argument("ChActuatorDynamics: stateful model required");
    ChExternalDynamicsODE::Initialize();
}

void ChActuatorDynamics::SetActuatorLength(double length, double velocity) {
    length_     = length;
    length_dot_ = velocity;
}

void ChActuatorDynamics::FreezeCommand(const ActuatorCommand& command) {
    command_ = command;
}

ActuatorState ChActuatorDynamics::BuildState(double time) const {
    ActuatorState s;
    s.time = time;
    if (attached_) {
        const ChVector3d p1 = local_ ? body1_->TransformPointLocalToParent(loc1_) : loc1_;
        const ChVector3d p2 = local_ ? body2_->TransformPointLocalToParent(loc2_) : loc2_;
        const ChVector3d d  = p2 - p1;
        s.displacement = d.Length();
        const ChVector3d dir = (s.displacement > 1e-12) ? d / s.displacement : ChVector3d(1, 0, 0);
        const ChVector3d l1_local = local_ ? loc1_ : body1_->TransformPointParentToLocal(loc1_);
        const ChVector3d l2_local = local_ ? loc2_ : body2_->TransformPointParentToLocal(loc2_);
        const ChVector3d v1 = body1_->PointSpeedLocalToParent(l1_local);
        const ChVector3d v2 = body2_->PointSpeedLocalToParent(l2_local);
        s.velocity = dir.Dot(v2 - v1);
    } else {
        s.displacement = length_;
        s.velocity     = length_dot_;
    }
    return s;
}

void ChActuatorDynamics::CalculateRHS(double                   time,
                                      const ChVectorDynamic<>& y,
                                      ChVectorDynamic<>&       rhs) {
    const ActuatorState state = BuildState(time);
    model_->CalculateRHS(time, y.data(), state, command_, rhs.data());
}

bool ChActuatorDynamics::CalculateJac(double                   time,
                                      const ChVectorDynamic<>& y,
                                      const ChVectorDynamic<>& /*rhs*/,
                                      ChMatrixDynamic<>&       jac) {
    const ActuatorState state = BuildState(time);
    return model_->CalculateJac(time, y.data(), state, command_, jac.data());
}

void ChActuatorDynamics::Update(double time, UpdateFlags update_flags) {
    ChExternalDynamicsODE::Update(time, update_flags);

    const auto& y = GetStates();
    const ActuatorState state = BuildState(time);
    effort_ = model_->EffortFromStates(y.data(), state);

    ActuatorTelemetry telem{};
    effort_ = ApplyPureEnvelope(effort_, state.velocity, envelope_, telem);
    telem.effort           = effort_;
    telem.mechanical_power = effort_ * state.velocity;
    telemetry_ = telem;

    if (attached_) {
        const ChVector3d p1 = local_ ? body1_->TransformPointLocalToParent(loc1_) : loc1_;
        const ChVector3d p2 = local_ ? body2_->TransformPointLocalToParent(loc2_) : loc2_;
        const ChVector3d d  = p2 - p1;
        const double len = d.Length();
        const ChVector3d dir = (len > 1e-12) ? d / len : ChVector3d(1, 0, 0);
        body1_->AccumulateForce(-effort_ * dir, p1, false);
        body2_->AccumulateForce( effort_ * dir, p2, false);
    }
}

}  // namespace actuators
}  // namespace chrono
