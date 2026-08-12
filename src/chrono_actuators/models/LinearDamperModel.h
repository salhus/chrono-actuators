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
// Linear viscous damper: F = -B*v.  Zero states.
// Used as PTO power-extraction baseline.
// =============================================================================

#ifndef CHRONO_ACTUATORS_MODEL_LINEAR_DAMPER_H
#define CHRONO_ACTUATORS_MODEL_LINEAR_DAMPER_H

#include "chrono_actuators/models/ActuatorModel.h"

namespace chrono {
namespace actuators {

/// Linear viscous damper: F = -B·v.
///
/// Sign convention: effort opposes velocity (energy extraction).  In terms
/// of the module convention (positive = extend), F is negative when v > 0.
/// Zero internal states; ComputeEffort is a pure function.
class LinearDamperModel : public ActuatorModel {
  public:
    /// @param damping_coefficient  Viscous damping coefficient B [N·s/m] or [N·m·s/rad].
    explicit LinearDamperModel(double damping_coefficient)
        : B_(damping_coefficient) {}

    double ComputeEffort(const ActuatorCommand& /*command*/,
                         const ActuatorState&   state) const override {
        return -B_ * state.velocity;
    }

    double GetDampingCoefficient() const { return B_; }

  private:
    double B_;
};

}  // namespace actuators
}  // namespace chrono

#endif  // CHRONO_ACTUATORS_MODEL_LINEAR_DAMPER_H
