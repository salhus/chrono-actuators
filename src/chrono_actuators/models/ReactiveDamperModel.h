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
// Reactive (spring + damper): F = -K*x - B*v.  Zero states.
// =============================================================================

#ifndef CHRONO_ACTUATORS_MODEL_REACTIVE_DAMPER_H
#define CHRONO_ACTUATORS_MODEL_REACTIVE_DAMPER_H

#include "chrono_actuators/models/ActuatorModel.h"

namespace chrono {
namespace actuators {

/// Reactive power-take-off: F = -K·x - B·v.
///
/// Both spring and damper terms oppose displacement/velocity.
/// Zero internal states; ComputeEffort is a pure function.
class ReactiveDamperModel : public ActuatorModel {
  public:
    struct Params {
        double stiffness = 0.0;  ///< spring constant K [N/m] or [N·m/rad]
        double damping   = 0.0;  ///< damping coefficient B [N·s/m] or [N·m·s/rad]
    };

    explicit ReactiveDamperModel(const Params& p) : p_(p) {}

    double ComputeEffort(const ActuatorCommand& /*command*/,
                         const ActuatorState&   state) const override {
        return -p_.stiffness * state.displacement - p_.damping * state.velocity;
    }

    const Params& GetParams() const { return p_; }

  private:
    Params p_;
};

}  // namespace actuators
}  // namespace chrono

#endif  // CHRONO_ACTUATORS_MODEL_REACTIVE_DAMPER_H
