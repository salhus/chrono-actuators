#include "chrono_actuators/ChActuatorAdapter.h"

namespace chrono {
namespace actuators {

ChActuatorAdapter::ChActuatorAdapter(std::shared_ptr<chrono::ChFunctionSetpointCallback> effort_callback)
    : effort_callback_(std::move(effort_callback)) {}

ChActuatorAdapter::ChActuatorAdapter(std::shared_ptr<chrono::ChLinkMotorLinearForce> linear_motor,
                                     std::shared_ptr<chrono::ChFunctionSetpointCallback> effort_callback)
    : linear_motor_(std::move(linear_motor)), effort_callback_(std::move(effort_callback)) {}

ChActuatorAdapter::ChActuatorAdapter(std::shared_ptr<chrono::ChLinkMotorRotationTorque> rotary_motor,
                                     std::shared_ptr<chrono::ChFunctionSetpointCallback> effort_callback)
    : rotary_motor_(std::move(rotary_motor)), effort_callback_(std::move(effort_callback)) {}

ChActuatorAdapter::ChActuatorAdapter(std::shared_ptr<chrono::ChShaftsMotorLoad> shaft_motor,
                                     std::shared_ptr<chrono::ChFunctionSetpointCallback> effort_callback)
    : shaft_motor_(std::move(shaft_motor)), effort_callback_(std::move(effort_callback)) {}

model::ActuatorState ChActuatorAdapter::ReadState(double time_seconds) const {
    model::ActuatorState state;
    state.time = time_seconds;
    state.measured_effort = last_effort_;

    // TODO(v0.2): marshal actual position/velocity/effort from wrapped Chrono
    // motor/shaft interfaces.
    return state;
}

void ChActuatorAdapter::ApplyEffort(model::Effort effort, double time_seconds) {
    last_effort_ = effort;

    // TODO(v0.2): push effort through ChFunctionSetpointCallback and concrete
    // link/shaft wrappers after model implementations land.
    (void)time_seconds;
}

}  // namespace actuators
}  // namespace chrono
