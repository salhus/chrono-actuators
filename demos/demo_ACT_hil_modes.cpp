// =============================================================================
// demo_ACT_hil_modes
//
// SIL / Parallel / HIL with a synthetic command source.
// Demonstrates watchdog, engage gate, and ramp-in.
// No ROS required.
// =============================================================================

#include <chrono>
#include <cstdio>
#include <thread>

#include "chrono_actuators/ChActuatorIO.h"
#include "chrono_actuators/models/LinearDamperModel.h"

int main() {
    using namespace chrono::actuators;

    std::printf("=== demo_ACT_hil_modes ===\n\n");

    ActuatorIOPolicy policy;
    policy.watchdog_timeout_s  = 0.2;
    policy.ramp_time_s         = 0.1;
    policy.model_effort_clamp  = 200.0;
    policy.output_effort_clamp = 150.0;

    ChActuatorIO io(policy);
    LinearDamperModel model(500.0);
    (void)model;

    ActuatorCommand cmd;
    cmd.effort  = 100.0;
    cmd.enabled = true;
    io.WriteCommand(cmd);

    std::printf("--- Phase 1: disengaged (expect zero effort) ---\n");
    for (int i = 0; i < 5; ++i) {
        const ActuatorCommand c = io.ReadCommand(i * 0.01);
        std::printf("  t=%.2f  effort=%.1f\n", i * 0.01, c.effort);
    }

    std::printf("\n--- Phase 2: engage (ramp over 0.1 s) ---\n");
    io.Engage(0.0);
    for (int i = 0; i <= 15; ++i) {
        const double t = i * 0.01;
        const ActuatorCommand c = io.ReadCommand(t);
        std::printf("  t=%.2f  effort=%.1f\n", t, c.effort);
    }

    std::printf("\n--- Phase 3: watchdog (stop writing, wait 0.25 s) ---\n");
    std::this_thread::sleep_for(std::chrono::milliseconds(250));
    {
        const ActuatorCommand c = io.ReadCommand(0.5);
        std::printf("  effort after timeout=%.1f (expect 0)\n", c.effort);
    }

    std::printf("\n--- Phase 4: output clamp (push 300 N, clamp at 150 N) ---\n");
    cmd.effort = 300.0;
    io.WriteCommand(cmd);
    {
        const ActuatorCommand c = io.ReadCommand(0.5);
        const double out = io.ApplyOutputClamp(c.effort);
        // c.effort is the model-level clamped value (300 N → 200 N by model_effort_clamp=200).
        // out is the final output after the independent output clamp (200 N → 150 N).
        std::printf("  commanded=%.1f  after_model_clamp=%.1f  after_output_clamp=%.1f\n",
                    cmd.effort, c.effort, out);
    }

    return 0;
}
