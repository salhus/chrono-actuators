#include <cassert>

#include "chrono_actuators/CommandStateCache.h"

int main() {
    chrono::actuators::CommandStateCache cache;

    chrono::actuators::model::ActuatorCommand cmd;
    cmd.effort_setpoint = 7.0;
    cmd.has_velocity_setpoint = true;
    cmd.velocity_setpoint = 3.5;
    cache.WriteCommand(cmd);

    const auto cmd_out = cache.ReadLatestCommand();
    assert(cmd_out.effort_setpoint == 7.0);
    assert(cmd_out.has_velocity_setpoint);
    assert(cmd_out.velocity_setpoint == 3.5);

    chrono::actuators::model::ActuatorState state;
    state.position = 1.25;
    state.measured_effort = 2.0;
    cache.WriteState(state);

    const auto state_out = cache.ReadLatestState();
    assert(state_out.position == 1.25);
    assert(state_out.measured_effort == 2.0);

    return 0;
}
