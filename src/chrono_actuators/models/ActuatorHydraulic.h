#ifndef CHRONO_ACTUATORS_MODEL_ACTUATOR_HYDRAULIC_H
#define CHRONO_ACTUATORS_MODEL_ACTUATOR_HYDRAULIC_H

#include "chrono_actuators/models/ActuatorModel.h"

namespace chrono {
namespace actuators {
namespace model {

/**
 * @brief Placeholder hydraulic actuator interface.
 *
 * TODO(v0.2): introduce pressure-flow state variables, valve dynamics,
 * and saturation/cavitation constraints.
 */
class ActuatorHydraulic : public ActuatorModel {
  public:
    ~ActuatorHydraulic() override = default;
    Effort ComputeEffort(const ActuatorCommand& command, const ActuatorState& state, double dt) override = 0;
};

}  // namespace model
}  // namespace actuators
}  // namespace chrono

#endif
