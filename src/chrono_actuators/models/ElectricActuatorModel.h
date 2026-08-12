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
// DC/BLDC electric actuator with electrical dynamics.
//
// State:  y[0] = winding current i [A]
// ODE:    di/dt = (V - R*i - Ke*omega) / L
//
// where V   = commanded voltage derived from effort command
//       omega = shaft angular velocity = state.velocity / gear_ratio
//
// Output torque:  tau = Kt * i * gear_ratio * eta_gear - B_vis * omega_out
//                     - Fc * sign(omega_out)
// clamped to [-tau_max, tau_max].
//
// The quasi-static voltage command is:
//   V = effort_cmd * R / Kt + Ke * omega  (current-loop approximation)
// so that at DC steady state i = effort_cmd / Kt and tau ≈ effort_cmd.
//
// Parameters are documented in SI units.  Parameterize to represent an
// ODrive-driven bench actuator after hardware identification.
// =============================================================================

#ifndef CHRONO_ACTUATORS_MODEL_ELECTRIC_ACTUATOR_H
#define CHRONO_ACTUATORS_MODEL_ELECTRIC_ACTUATOR_H

#include <algorithm>
#include <cmath>
#include <stdexcept>

#include "chrono_actuators/models/ActuatorModel.h"

namespace chrono {
namespace actuators {

/// Parameter set for a geared DC/BLDC electric actuator.
/// All quantities in SI base units.
struct ElectricActuatorParams {
    // Electrical
    double R  = 1.0;    ///< winding resistance [Ω]
    double L  = 1e-3;   ///< winding inductance [H] (small → stiff ODE)
    double Kt = 0.1;    ///< torque constant [N·m/A]
    double Ke = 0.1;    ///< back-EMF constant [V·s/rad]

    // Mechanical
    double gear_ratio      = 1.0;   ///< N = omega_motor / omega_output
    double gear_efficiency = 1.0;   ///< η ∈ (0,1]; same for both directions

    // Friction
    double B_viscous = 0.0;  ///< viscous friction at output shaft [N·m·s/rad]
    double F_coulomb = 0.0;  ///< Coulomb friction at output shaft [N·m]

    // Limits
    double current_max = 1e30;  ///< peak current limit [A]
    double effort_max  = 1e30;  ///< peak output torque/force limit [N·m] or [N]

    // Initial condition
    double i0 = 0.0;  ///< initial winding current [A]
};

/// DC/BLDC electric actuator model.
///
/// Internal state: y[0] = winding current i [A].
/// IsStiff() returns true because L is typically small (< 1 ms).
/// An analytic Jacobian is provided so Chrono's implicit solver remains efficient.
class ElectricActuatorModel : public ActuatorModel {
  public:
    explicit ElectricActuatorModel(const ElectricActuatorParams& p) : p_(p) {
        if (p_.L <= 0.0)
            throw std::invalid_argument("ElectricActuatorModel: inductance L must be > 0");
        if (p_.R <= 0.0)
            throw std::invalid_argument("ElectricActuatorModel: resistance R must be > 0");
        if (p_.gear_ratio <= 0.0)
            throw std::invalid_argument("ElectricActuatorModel: gear_ratio must be > 0");
        if (p_.gear_efficiency <= 0.0 || p_.gear_efficiency > 1.0)
            throw std::invalid_argument("ElectricActuatorModel: gear_efficiency must be in (0,1]");
    }

    const ElectricActuatorParams& GetParams() const { return p_; }

    // -------------------------------------------------------------------------
    // ActuatorModel interface
    // -------------------------------------------------------------------------

    /// Quasi-static effort: uses current-state y[0] when called from
    /// EffortFromStates.  When GetNumStates() > 0 this is only called
    /// inside ChActuatorDynamics after each accepted step.
    double ComputeEffort(const ActuatorCommand& command,
                         const ActuatorState&   state) const override {
        const double omega_out = state.velocity;
        const double omega_motor = omega_out * p_.gear_ratio;
        const double V = CommandToVoltage(command.effort, omega_motor);
        const double i = std::clamp((V - p_.Ke * omega_motor) / p_.R,
                                    -p_.current_max, p_.current_max);
        return OutputEffort(i, omega_out);
    }

    unsigned int GetNumStates() const override { return 1; }

    void SetInitialConditions(double* y0) const override {
        y0[0] = p_.i0;
    }

    void CalculateRHS(double /*time*/,
                      const double*          y,
                      const ActuatorState&   state,
                      const ActuatorCommand& command,
                      double*                rhs) const override {
        const double i = y[0];
        const double omega_motor = state.velocity * p_.gear_ratio;
        const double V = CommandToVoltage(command.effort, omega_motor);
        rhs[0] = (V - p_.R * i - p_.Ke * omega_motor) / p_.L;
    }

    bool CalculateJac(double /*time*/,
                      const double*          /*y*/,
                      const ActuatorState&   /*state*/,
                      const ActuatorCommand& /*command*/,
                      double*                jac) const override {
        // d(rhs[0])/d(y[0]) = -R/L
        jac[0] = -p_.R / p_.L;
        return true;
    }

    bool IsStiff() const override { return true; }

    double EffortFromStates(const double*        y,
                            const ActuatorState& state) const override {
        const double i = y[0];
        return OutputEffort(std::clamp(i, -p_.current_max, p_.current_max),
                            state.velocity);
    }

  private:
    /// Translate effort command (desired output torque) to motor voltage.
    double CommandToVoltage(double effort_cmd, double omega_motor) const {
        const double i_des = effort_cmd / (p_.Kt * p_.gear_ratio * p_.gear_efficiency);
        return p_.R * std::clamp(i_des, -p_.current_max, p_.current_max)
               + p_.Ke * omega_motor;
    }

    /// Compute output effort from motor current and output velocity.
    double OutputEffort(double i, double omega_out) const {
        double tau = p_.Kt * i * p_.gear_ratio * p_.gear_efficiency;
        tau -= p_.B_viscous * omega_out;
        if (omega_out > 0.0)
            tau -= p_.F_coulomb;
        else if (omega_out < 0.0)
            tau += p_.F_coulomb;
        return std::clamp(tau, -p_.effort_max, p_.effort_max);
    }

    ElectricActuatorParams p_;
};

}  // namespace actuators
}  // namespace chrono

#endif  // CHRONO_ACTUATORS_MODEL_ELECTRIC_ACTUATOR_H
