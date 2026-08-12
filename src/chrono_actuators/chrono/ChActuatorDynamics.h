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
// Chrono binding: monolithic ODE integration via ChExternalDynamicsODE.
//
// Models with GetNumStates() > 0 must use this binding so that internal states
// (e.g., winding current) are integrated simultaneously with the multibody
// system and benefit from Jacobian assembly, step-size control, and implicit
// solvers.
//
// This design mirrors ChHydraulicActuator (Radu Serban) in:
//   src/chrono/physics/ChHydraulicActuator.{h,cpp}
// and supports the same attached/standalone duality:
//   - Attached:   construct with body pair + points
//   - Standalone: construct standalone + SetActuatorLength(length, velocity)
// =============================================================================

#ifndef CHRONO_ACTUATORS_CHRONO_DYNAMICS_H
#define CHRONO_ACTUATORS_CHRONO_DYNAMICS_H

#include <memory>

#include "chrono/physics/ChBody.h"
#include "chrono/physics/ChExternalDynamicsODE.h"
#include "chrono_actuators/ChApiActuators.h"
#include "chrono_actuators/models/ActuatorModel.h"
#include "chrono_actuators/models/Envelope.h"
#include "chrono_actuators/models/Telemetry.h"

namespace chrono {
namespace actuators {

/// Monolithic ODE actuator: wraps a stateful ActuatorModel in ChExternalDynamicsODE.
///
/// The model's internal states are integrated simultaneously with the
/// multibody system — no external sub-stepping, no stability issues with
/// stiff electrical/hydraulic dynamics.
///
/// **Solver requirement for stiff models**
/// If the model's `IsStiff()` returns true (e.g. `ElectricActuatorModel`),
/// `ChExternalDynamicsODE::InjectKRMMatrices()` inserts a KRM block into the
/// system descriptor.  The default `ChSystemNSC` PSOR solver is an iterative
/// VI solver that **cannot** consume KRM blocks and will abort with a
/// `std::runtime_error`.  Before calling `sys.Add(this)`, configure:
/// @code{.cpp}
///   auto solver = chrono_types::make_shared<ChSolverSparseLU>(); // or ChSolverSparseQR
///   sys.SetSolver(solver);
///   solver->UseSparsityPatternLearner(true);
///   solver->LockSparsityPattern(true);
///   sys.SetTimestepperType(ChTimestepper::Type::EULER_IMPLICIT);
/// @endcode
/// This requirement does not apply to zero-state or non-stiff models.
///
/// Call Initialize() explicitly after construction and before adding this item
/// to a ChSystem. This mirrors Chrono's ChHydraulicActuatorBase and avoids a
/// virtual call to GetNumStates() during base-class construction.
///
/// Applied force/torque is extracted via EffortFromStates() after each
/// accepted step and loaded into Chrono residuals for the attached mode, or
/// returned via GetEffort() for standalone use.
class ChApiActuators ChActuatorDynamics : public chrono::ChExternalDynamicsODE {
  public:
    /// Attached mode: actuator between two bodies.
    /// @param model    Stateful ActuatorModel (GetNumStates() > 0).
    /// @param body1    First attachment body.
    /// @param body2    Second attachment body.
    /// @param local    True if loc1/loc2 are in body-local frames.
    /// @param loc1     Attachment point on body1.
    /// @param loc2     Attachment point on body2.
    ChActuatorDynamics(std::shared_ptr<ActuatorModel> model,
                       std::shared_ptr<ChBody>        body1,
                       std::shared_ptr<ChBody>        body2,
                       bool                           local,
                       const ChVector3d&             loc1,
                       const ChVector3d&             loc2,
                       const EnvelopeParams&         envelope = EnvelopeParams{});

    /// Standalone mode: no bodies; caller provides kinematics via SetActuatorLength.
    explicit ChActuatorDynamics(std::shared_ptr<ActuatorModel> model,
                                const EnvelopeParams&         envelope = EnvelopeParams{});

    /// Standalone mode: set current actuator length and rate (co-simulation use).
    void SetActuatorLength(double length, double velocity);

    /// Initialize the actuator after construction and before sys.Add(...).
    void Initialize() override;

    /// Update frozen command for the current step.
    void FreezeCommand(const ActuatorCommand& command);

    /// Applied effort after the last accepted step [N] or [N·m].
    double GetEffort() const { return effort_; }

    const ActuatorTelemetry& GetTelemetry() const { return telemetry_; }

    unsigned int GetNumStates() const override { return model_->GetNumStates(); }

    void SetInitialConditions(ChVectorDynamic<>& y0) override {
        model_->SetInitialConditions(y0.data());
    }

    void CalculateRHS(double                   time,
                      const ChVectorDynamic<>& y,
                      ChVectorDynamic<>&       rhs) override;

    bool CalculateJac(double                   time,
                      const ChVectorDynamic<>& y,
                      const ChVectorDynamic<>& rhs,
                      ChMatrixDynamic<>&       jac) override;

    bool IsStiff() const override { return model_->IsStiff(); }

    void Update(double time, UpdateFlags update_flags) override;

  private:
    void IntLoadResidual_F(const unsigned int off, ChVectorDynamic<>& R, const double c) override;

    void VariablesFbLoadForces(double factor = 1) override;

    struct GeometryState {
        ChVector3d p1{0, 0, 0};
        ChVector3d p2{0, 0, 0};
        ChVector3d dir{1, 0, 0};
        double     length{0.0};
        double     rate{0.0};
    };

    GeometryState BuildGeometry() const;
    ActuatorState BuildState(double time) const;

    std::shared_ptr<ActuatorModel> model_;
    EnvelopeParams                 envelope_;

    std::shared_ptr<ChBody> body1_;
    std::shared_ptr<ChBody> body2_;
    bool                    local_{false};
    ChVector3d              loc1_{};
    ChVector3d              loc2_{};
    bool                    attached_{false};

    double length_{0.0};
    double length_dot_{0.0};

    ActuatorCommand   command_;
    double            effort_{0.0};
    ActuatorTelemetry telemetry_;
    ChVectorDynamic<> m_Qforce;
};

}  // namespace actuators
}  // namespace chrono

#endif  // CHRONO_ACTUATORS_CHRONO_DYNAMICS_H
