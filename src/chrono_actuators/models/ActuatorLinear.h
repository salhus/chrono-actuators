#ifndef CHRONO_ACTUATORS_MODEL_ACTUATOR_LINEAR_H
#define CHRONO_ACTUATORS_MODEL_ACTUATOR_LINEAR_H

#include "chrono_actuators/models/ActuatorModel.h"

namespace chrono {
namespace actuators {
namespace model {

/**
 * @brief Placeholder linear actuator model interface.
 *
 * TODO(v0.2): implement quasi-static and optional dynamic terms for
 * translational force-producing actuators.
 */
class ActuatorLinear : public ActuatorModel {
  public:
    ~ActuatorLinear() override = default;
    Effort ComputeEffort(const ActuatorCommand& command, const ActuatorState& state, double dt) override = 0;
};

}  // namespace model
}  // namespace actuators
}  // namespace chrono

#endif
