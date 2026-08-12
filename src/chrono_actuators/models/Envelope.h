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
// Composable effort envelope applied on the per-step path.
//
// Pure operations (no state, re-query safe):
//   Saturation, deadband, quadrant efficiency.
//
// Step-scoped operation (stores one double of state):
//   Rate limiting.  Apply AFTER ComputeEffort, BEFORE writing to physics.
//   Do NOT call inside a force functor — that would violate Invariant A.
// =============================================================================

#ifndef CHRONO_ACTUATORS_MODEL_ENVELOPE_H
#define CHRONO_ACTUATORS_MODEL_ENVELOPE_H

#include <algorithm>
#include <cmath>
#include <stdexcept>

#include "chrono_actuators/models/Telemetry.h"

namespace chrono {
namespace actuators {

/// Symmetric or asymmetric effort saturation limits [N] or [N·m].
struct SaturationLimits {
    double max_extend  =  1e30; ///< upper bound (positive direction)
    double max_retract = -1e30; ///< lower bound (negative direction), must be <= 0
};

/// Quadrant-dependent efficiency factors (dimensionless, [0,1]).
///
/// Four quadrants arise from sign(effort) × sign(velocity):
///   (+,+) motoring-extend   (+,−) generating-extend
///   (−,−) motoring-retract  (−,+) generating-retract
struct QuadrantEfficiency {
    double motoring   = 1.0;   ///< effort and velocity same sign
    double generating = 1.0;   ///< effort and velocity opposite sign
};

/// Envelope parameters.  All fields have safe defaults (no-op).
struct EnvelopeParams {
    SaturationLimits   saturation{};
    double             deadband    = 0.0;   ///< symmetric deadband half-width [N] or [N·m]
    double             max_rate    = 1e30;  ///< effort rate limit [N/s] or [N·m/s]
    QuadrantEfficiency efficiency{};
    double             viscous_friction = 0.0;   ///< coefficient, multiplied by velocity
    double             coulomb_friction = 0.0;   ///< constant opposing-motion friction
};

/// Stateless envelope: pure operations only.
/// Apply saturation, deadband, and quadrant efficiency.
/// Returns the modified effort and updates telemetry flags.
inline double ApplyPureEnvelope(double effort,
                                double velocity,
                                const EnvelopeParams& params,
                                ActuatorTelemetry&    telem) {
    if (params.deadband < 0.0)
        throw std::invalid_argument("ApplyPureEnvelope: deadband must be >= 0");
    if (params.efficiency.motoring < 0.0 || params.efficiency.motoring > 1.0 ||
        params.efficiency.generating < 0.0 || params.efficiency.generating > 1.0) {
        throw std::invalid_argument("ApplyPureEnvelope: efficiencies must be in [0,1]");
    }
    if (params.saturation.max_retract > params.saturation.max_extend)
        throw std::invalid_argument("ApplyPureEnvelope: saturation bounds are inverted");

    // --- deadband ---
    if (std::abs(effort) <= params.deadband) {
        effort = 0.0;
    }

    // --- friction (opposing motion, pure function of velocity) ---
    double friction = params.viscous_friction * velocity;
    if (velocity > 0.0)
        friction += params.coulomb_friction;
    else if (velocity < 0.0)
        friction -= params.coulomb_friction;
    effort -= friction;

    // --- quadrant efficiency ---
    const bool same_sign = (effort >= 0.0) == (velocity >= 0.0);
    const double eta = same_sign ? params.efficiency.motoring
                                 : params.efficiency.generating;
    effort *= eta;
    telem.efficiency = eta;

    // --- saturation ---
    if (effort > params.saturation.max_extend) {
        effort = params.saturation.max_extend;
        telem.effort_saturated = true;
    } else if (effort < params.saturation.max_retract) {
        effort = params.saturation.max_retract;
        telem.effort_saturated = true;
    }

    return effort;
}

/// Step-scoped rate limiter.  Call once per accepted step, NOT inside a functor.
/// prev_effort is updated in-place.
inline double ApplyRateLimit(double effort,
                             double& prev_effort,
                             double dt,
                             double max_rate,
                             ActuatorTelemetry& telem) {
    if (dt <= 0.0 || max_rate >= 1e29)
        return effort;
    const double delta_max = max_rate * dt;
    const double delta = effort - prev_effort;
    if (std::abs(delta) > delta_max) {
        effort = prev_effort + std::copysign(delta_max, delta);
        telem.rate_limited = true;
    }
    prev_effort = effort;
    return effort;
}

}  // namespace actuators
}  // namespace chrono

#endif  // CHRONO_ACTUATORS_MODEL_ENVELOPE_H
