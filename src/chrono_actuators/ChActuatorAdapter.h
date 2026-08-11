#ifndef CHRONO_ACTUATORS_CH_ACTUATOR_ADAPTER_H
#define CHRONO_ACTUATORS_CH_ACTUATOR_ADAPTER_H

#include <memory>

#include "chrono/functions/ChFunctionSetpoint.h"
#include "chrono/physics/ChLinkMotorLinearForce.h"
#include "chrono/physics/ChLinkMotorRotationTorque.h"
#include "chrono/physics/ChShaftsMotorLoad.h"
#include "chrono_actuators/ChApiActuators.h"
#include "chrono_actuators/models/ActuatorModel.h"

namespace chrono {
namespace actuators {

/**
 * @brief Chrono-aware adapter boundary for actuator models.
 *
 * This is the only layer allowed to touch Chrono link/shaft motor types.
 * Model classes remain Chrono-free and exchange plain C++ structs.
 *
 * Future note: a solver-integrated `ChLinkMotorActuator` variant could be
 * considered in a later major revision.
 */
class ChApiActuators ChActuatorAdapter {
  public:
    ChActuatorAdapter() = default;
    explicit ChActuatorAdapter(std::shared_ptr<chrono::ChFunctionSetpointCallback> effort_callback);

    ChActuatorAdapter(std::shared_ptr<chrono::ChLinkMotorLinearForce> linear_motor,
                      std::shared_ptr<chrono::ChFunctionSetpointCallback> effort_callback);

    ChActuatorAdapter(std::shared_ptr<chrono::ChLinkMotorRotationTorque> rotary_motor,
                      std::shared_ptr<chrono::ChFunctionSetpointCallback> effort_callback);

    ChActuatorAdapter(std::shared_ptr<chrono::ChShaftsMotorLoad> shaft_motor,
                      std::shared_ptr<chrono::ChFunctionSetpointCallback> effort_callback);

    model::ActuatorState ReadState(double time_seconds) const;
    void ApplyEffort(model::Effort effort, double time_seconds);

  private:
    std::shared_ptr<chrono::ChLinkMotorLinearForce> linear_motor_;
    std::shared_ptr<chrono::ChLinkMotorRotationTorque> rotary_motor_;
    std::shared_ptr<chrono::ChShaftsMotorLoad> shaft_motor_;
    std::shared_ptr<chrono::ChFunctionSetpointCallback> effort_callback_;
    model::Effort last_effort_ = 0.0;
};

}  // namespace actuators
}  // namespace chrono

#endif
