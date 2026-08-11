#include "chrono_actuators/ChActuator.h"

namespace chrono {
namespace actuators {

ChActuator::ChActuator(std::unique_ptr<model::ActuatorModel> model, std::shared_ptr<ChActuatorAdapter> adapter)
    : model_(std::move(model)), adapter_(std::move(adapter)) {}

void ChActuator::SetCommand(const model::ActuatorCommand& command) noexcept {
    cache_.WriteCommand(command);
}

model::ActuatorState ChActuator::GetState() const noexcept {
    return cache_.ReadLatestState();
}

void ChActuator::Advance(double dt) {
    if (!model_ || !adapter_) {
        // TODO(v0.2): introduce diagnostics/error reporting for null runtime wiring.
        return;
    }

    const auto command = cache_.ReadLatestCommand();
    const auto input_state = adapter_->ReadState(simulation_time_seconds_);

    model_->AdvanceInternalState(dt);
    const auto effort = model_->ComputeEffort(command, input_state, dt);

    adapter_->ApplyEffort(effort, simulation_time_seconds_);

    auto output_state = input_state;
    output_state.measured_effort = effort;
    cache_.WriteState(output_state);

    simulation_time_seconds_ += dt;
}

}  // namespace actuators
}  // namespace chrono
