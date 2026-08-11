#ifndef CHRONO_ACTUATORS_CH_ACTUATOR_H
#define CHRONO_ACTUATORS_CH_ACTUATOR_H

#include <memory>

#include "chrono_actuators/ChActuatorAdapter.h"
#include "chrono_actuators/ChApiActuators.h"
#include "chrono_actuators/CommandStateCache.h"
#include "chrono_actuators/models/ActuatorModel.h"

namespace chrono {
namespace actuators {

/**
 * @brief Chrono-facing actuator orchestration wrapper.
 *
 * Runtime flow per physics step:
 * 1) non-blocking command read from CommandStateCache
 * 2) state marshalling from ChActuatorAdapter
 * 3) effort computation in engine-neutral ActuatorModel
 * 4) effort write-back through ChActuatorAdapter
 * 5) non-blocking state publish to CommandStateCache
 */
class ChApiActuators ChActuator {
  public:
    ChActuator(std::unique_ptr<model::ActuatorModel> model, std::shared_ptr<ChActuatorAdapter> adapter);

    void SetCommand(const model::ActuatorCommand& command) noexcept;
    model::ActuatorState GetState() const noexcept;

    /**
     * @brief Execute one simulation step of actuator logic.
     */
    void Advance(double dt);

  private:
    std::unique_ptr<model::ActuatorModel> model_;
    std::shared_ptr<ChActuatorAdapter> adapter_;
    CommandStateCache cache_;
    double simulation_time_seconds_ = 0.0;
};

}  // namespace actuators
}  // namespace chrono

#endif
