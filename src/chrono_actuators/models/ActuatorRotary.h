#ifndef CHRONO_ACTUATORS_MODEL_ACTUATOR_ROTARY_H
#define CHRONO_ACTUATORS_MODEL_ACTUATOR_ROTARY_H

#include "chrono_actuators/models/ActuatorModel.h"

namespace chrono {
namespace actuators {
namespace model {

/**
 * @brief Placeholder rotary motor/generator model interface.
 *
 * TODO(v0.2): implement bidirectional electromechanical rotary behavior.
 */
class ActuatorRotary : public ActuatorModel {
  public:
    ~ActuatorRotary() override = default;
    Effort ComputeEffort(const ActuatorCommand& command, const ActuatorState& state, double dt) override = 0;
};

}  // namespace model
}  // namespace actuators
}  // namespace chrono

#endif
