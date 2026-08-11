#include <iostream>
#include <memory>

#include "chrono/physics/ChSystemNSC.h"
#include "chrono_actuators/ChActuator.h"

class DemoModel final : public chrono::actuators::model::ActuatorModel {
  public:
    chrono::actuators::model::Effort ComputeEffort(const chrono::actuators::model::ActuatorCommand& command,
                                                   const chrono::actuators::model::ActuatorState&,
                                                   double) override {
        // TODO(v0.2): replace with concrete actuator model implementation.
        return command.effort_setpoint;
    }
};

int main() {
    chrono::ChSystemNSC system;
    (void)system;  // Keeps demo Chrono-linked while scaffold has no bound motor yet.

    auto model = std::make_unique<DemoModel>();
    auto adapter = std::make_shared<chrono::actuators::ChActuatorAdapter>();
    chrono::actuators::ChActuator actuator(std::move(model), adapter);

    chrono::actuators::model::ActuatorCommand command;
    command.effort_setpoint = 12.5;
    actuator.SetCommand(command);

    for (int i = 0; i < 5; ++i) {
        actuator.Advance(0.01);
        const auto state = actuator.GetState();
        std::cout << "step=" << i << " t=" << state.time << " effort=" << state.measured_effort << '\n';
    }

    // Intended v0.1 follow-up demo scenarios: sphere-heave and 1-DOF pendulum.
    return 0;
}
