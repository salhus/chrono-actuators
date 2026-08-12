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

#include <iostream>
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
}

ChActuatorDynamics::ChActuatorDynamics(std::shared_ptr<ActuatorModel> model,
                                       const EnvelopeParams&          envelope)
    : model_(std::move(model)), envelope_(envelope), attached_(false) {
    if (!model_)
        throw std::invalid_argument("ChActuatorDynamics: model must not be null");
}

void ChActuatorDynamics::Initialize() {
    if (!model_)
        throw std::invalid_argument("ChActuatorDynamics: model must not be null");
    if (model_->GetNumStates() == 0)
        throw std::invalid_argument("ChActuatorDynamics: stateful model required");

    // Warn when a stiff model is initialized without a direct solver.
    // The default ChSystemNSC PSOR solver is an iterative VI solver that cannot
    // consume KRM blocks.  Stiff models (IsStiff() == true) cause
    // ChExternalDynamicsODE::InjectKRMMatrices() to insert a KRM block, which
    // will cause the PSOR solver to abort with a runtime_error.
    //
    // Required setup (call before sys.Add(this)):
    //   auto solver = chrono_types::make_shared<ChSolverSparseLU>();   // or ChSolverSparseQR
    //   sys.SetSolver(solver);
    //   sys.SetTimestepperType(ChTimestepper::Type::EULER_IMPLICIT);
    //
    // See demos/demo_ACT_electric.cpp for a complete example.
    if (model_->IsStiff()) {
        std::cerr << "[ChActuatorDynamics] WARNING: model IsStiff() == true.\n"
                  << "  The default ChSystemNSC PSOR solver cannot consume KRM blocks and will abort.\n"
                  << "  Configure a direct solver (e.g. ChSolverSparseLU) and an implicit timestepper\n"
                  << "  (e.g. ChTimestepper::Type::EULER_IMPLICIT) before calling sys.Add(this).\n"
                  << "  See demos/demo_ACT_electric.cpp for the required setup pattern.\n";
    }

    ChExternalDynamicsODE::Initialize();

    if (attached_) {
        m_Qforce.resize(12);
        m_Qforce.setZero();
    }
}

void ChActuatorDynamics::SetActuatorLength(double length, double velocity) {
    length_     = length;
    length_dot_ = velocity;
}

void ChActuatorDynamics::FreezeCommand(const ActuatorCommand& command) {
    command_ = command;
}

ChActuatorDynamics::GeometryState ChActuatorDynamics::BuildGeometry() const {
    GeometryState g;

    if (!attached_) {
        g.length = length_;
        g.rate   = length_dot_;
        return g;
    }

    g.p1 = local_ ? body1_->TransformPointLocalToParent(loc1_) : loc1_;
    g.p2 = local_ ? body2_->TransformPointLocalToParent(loc2_) : loc2_;

    const ChVector3d delta = g.p1 - g.p2;
    g.length = delta.Length();
    g.dir    = (g.length > 1e-12) ? delta / g.length : ChVector3d(1, 0, 0);

    const ChVector3d l1_local = local_ ? loc1_ : body1_->TransformPointParentToLocal(loc1_);
    const ChVector3d l2_local = local_ ? loc2_ : body2_->TransformPointParentToLocal(loc2_);
    const ChVector3d v1       = body1_->PointSpeedLocalToParent(l1_local);
    const ChVector3d v2       = body2_->PointSpeedLocalToParent(l2_local);
    g.rate = g.dir.Dot(v1 - v2);

    return g;
}

ActuatorState ChActuatorDynamics::BuildState(double time) const {
    ActuatorState s;
    s.time = time;

    const GeometryState g = BuildGeometry();
    s.displacement = g.length;
    s.velocity     = g.rate;

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
        const GeometryState g = BuildGeometry();
        const ChVector3d force = effort_ * g.dir;

        const ChVector3d atorque1 = Vcross(g.p1 - body1_->GetPos(), force);
        const ChVector3d ltorque1 = body1_->TransformDirectionParentToLocal(atorque1);
        m_Qforce.segment(0, 3) = force.eigen();
        m_Qforce.segment(3, 3) = ltorque1.eigen();

        const ChVector3d atorque2 = Vcross(g.p2 - body2_->GetPos(), -force);
        const ChVector3d ltorque2 = body2_->TransformDirectionParentToLocal(atorque2);
        m_Qforce.segment(6, 3) = (-force).eigen();
        m_Qforce.segment(9, 3) = ltorque2.eigen();
    }
}

void ChActuatorDynamics::IntLoadResidual_F(const unsigned int off, ChVectorDynamic<>& R, const double c) {
    if (!IsActive())
        return;

    ChExternalDynamicsODE::IntLoadResidual_F(off, R, c);

    if (!attached_)
        return;

    if (body1_->Variables().IsActive()) {
        R.segment(body1_->Variables().GetOffset() + 0, 3) += c * m_Qforce.segment(0, 3);
        R.segment(body1_->Variables().GetOffset() + 3, 3) += c * m_Qforce.segment(3, 3);
    }

    if (body2_->Variables().IsActive()) {
        R.segment(body2_->Variables().GetOffset() + 0, 3) += c * m_Qforce.segment(6, 3);
        R.segment(body2_->Variables().GetOffset() + 3, 3) += c * m_Qforce.segment(9, 3);
    }
}

void ChActuatorDynamics::VariablesFbLoadForces(double factor) {
    ChExternalDynamicsODE::VariablesFbLoadForces(factor);

    if (!attached_)
        return;

    body1_->Variables().Force().segment(0, 3) += factor * m_Qforce.segment(0, 3);
    body1_->Variables().Force().segment(3, 3) += factor * m_Qforce.segment(3, 3);
    body2_->Variables().Force().segment(0, 3) += factor * m_Qforce.segment(6, 3);
    body2_->Variables().Force().segment(3, 3) += factor * m_Qforce.segment(9, 3);
}

}  // namespace actuators
}  // namespace chrono
