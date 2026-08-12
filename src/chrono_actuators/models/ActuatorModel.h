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
// Engine-neutral actuator model interface.
//
// Sign convention (stated here; referenced at every binding boundary):
//   Positive effort acts to extend (linear) or rotate in the positive-angle
//   direction (rotary), matching Chrono's motor convention.
//   Downstream consumers whose force opposes motion (e.g. SEA-Stack IPTOModel)
//   must negate in a thin shim.
// =============================================================================

#ifndef CHRONO_ACTUATORS_MODEL_ACTUATOR_MODEL_H
#define CHRONO_ACTUATORS_MODEL_ACTUATOR_MODEL_H

namespace chrono {
namespace actuators {

/// Actuation intent.  Effort is the primary channel; position/velocity intent
/// is advisory and consumed by an external control law only.
struct ActuatorCommand {
    double effort   = 0.0;   ///< commanded force [N] or torque [N·m]
    double position = 0.0;   ///< advisory position/angle setpoint [m] or [rad]
    double velocity = 0.0;   ///< advisory velocity setpoint [m/s] or [rad/s]
    bool   enabled  = false; ///< actuator enabled flag
};

/// Measured/derived actuator kinematic state at evaluation time.
struct ActuatorState {
    double displacement = 0.0; ///< relative extension [m] or angle [rad]
    double velocity     = 0.0; ///< relative rate [m/s] or [rad/s]
    double time         = 0.0; ///< current simulation time [s]
};

/// Base interface for all engine-neutral actuator models.
///
/// Two evaluation contracts:
///   GetNumStates() == 0  →  bind via functor adapters (re-query safe, const)
///   GetNumStates()  > 0  →  bind via ChActuatorDynamics : ChExternalDynamicsODE
///
/// ComputeEffort() is declared const to enforce Invariant A: a zero-state model
/// MUST be a pure function of (command, state, time).  Implicit integrators
/// (HHT, EULER_IMPLICIT) call force functors multiple times at the same
/// simulation time within one step; a mutable ComputeEffort would silently
/// produce wrong results.
class ActuatorModel {
  public:
    virtual ~ActuatorModel() = default;

    /// Compute output effort from command and kinematic state.
    ///
    /// @param command  Frozen command snapshot (constant within a step per Invariant C).
    /// @param state    Current kinematic state.
    /// @return         Effort [N] or torque [N·m], positive = extend/positive-rotation.
    ///
    /// MUST be a pure function of its arguments when GetNumStates() == 0.
    /// No accumulation, no mutation of member state, no I/O.
    virtual double ComputeEffort(const ActuatorCommand& command,
                                 const ActuatorState&   state) const = 0;

    /// Number of internal ODE states.  0 = algebraic; >0 requires ChActuatorDynamics.
    virtual unsigned int GetNumStates() const { return 0; }

    /// Populate y0 (size == GetNumStates()).  Called once before integration begins.
    virtual void SetInitialConditions(double* /*y0*/) const {}

    /// Evaluate ODE right-hand side: dy/dt = rhs(t, y, state, command).
    /// Called by ChActuatorDynamics during Chrono's monolithic integration.
    virtual void CalculateRHS(double               /*time*/,
                              const double*        /*y*/,
                              const ActuatorState& /*state*/,
                              const ActuatorCommand& /*command*/,
                              double*              /*rhs*/) const {}

    /// Provide analytic Jacobian d(rhs)/d(y).  Return false → finite-difference fallback.
    virtual bool CalculateJac(double               /*time*/,
                              const double*        /*y*/,
                              const ActuatorState& /*state*/,
                              const ActuatorCommand& /*command*/,
                              double*              /*jac*/) const { return false; }

    /// True when the ODE is stiff.  Controls whether Chrono picks an implicit sub-solver.
    virtual bool IsStiff() const { return false; }

    /// Extract effort from integrated states (called after each accepted step).
    /// Only meaningful when GetNumStates() > 0.
    virtual double EffortFromStates(const double*        /*y*/,
                                    const ActuatorState& /*state*/) const { return 0.0; }
};

}  // namespace actuators
}  // namespace chrono

#endif  // CHRONO_ACTUATORS_MODEL_ACTUATOR_MODEL_H
