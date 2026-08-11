#ifndef CHRONO_ACTUATORS_CONTROLLER_BASE_H
#define CHRONO_ACTUATORS_CONTROLLER_BASE_H

#include "chrono_actuators/models/ActuatorModel.h"

namespace chrono {
namespace actuators {

/**
 * @brief Generic controller setpoint payload.
 */
struct Setpoint {
    bool has_position = false;
    double position = 0.0;
    bool has_velocity = false;
    double velocity = 0.0;
    bool has_effort_ff = false;
    double effort_feedforward = 0.0;
};

/**
 * @brief Controller interface that maps a setpoint + state to effort.
 *
 * Implementations are intentionally out-of-tree and belong to a separate
 * controls-library repository. This project must not depend on that repo.
 */
class ControllerBase {
  public:
    virtual ~ControllerBase() = default;
    virtual model::Effort compute(const Setpoint& setpoint, const model::ActuatorState& state) = 0;
};

}  // namespace actuators
}  // namespace chrono

#endif
