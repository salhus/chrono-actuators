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
// Per-step actuator telemetry.
// =============================================================================

#ifndef CHRONO_ACTUATORS_MODEL_TELEMETRY_H
#define CHRONO_ACTUATORS_MODEL_TELEMETRY_H

#include <cmath>

namespace chrono {
namespace actuators {

/// Per-step actuator telemetry produced after ComputeEffort.
struct ActuatorTelemetry {
    double effort           = 0.0;          ///< applied effort [N] or [N·m]
    double mechanical_power = 0.0;          ///< effort × velocity [W]
    double electrical_power = NAN;          ///< NaN when not modeled
    double efficiency       = 0.0;          ///< dimensionless [0,1]
    bool   effort_saturated = false;        ///< true when effort clamp was active
    bool   rate_limited     = false;        ///< true when rate limiter was active
};

}  // namespace actuators
}  // namespace chrono

#endif  // CHRONO_ACTUATORS_MODEL_TELEMETRY_H
