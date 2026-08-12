// =============================================================================
// Test: ChActuatorIO — seqlock, watchdog, engage gate, ramp, clamps.
// No Chrono dependency.
// =============================================================================

#include <atomic>
#include <cassert>
#include <chrono>
#include <cmath>
#include <thread>

#include "chrono_actuators/ChActuatorIO.h"

using namespace chrono::actuators;

// ---------------------------------------------------------------------------
// Engage gate: disengaged → zero effort
// ---------------------------------------------------------------------------
static void test_engage_gate() {
    ActuatorIOPolicy policy;
    policy.watchdog_timeout_s = 100.0;
    policy.ramp_time_s        = 0.0;

    ChActuatorIO io(policy);

    ActuatorCommand cmd;
    cmd.effort  = 200.0;
    cmd.enabled = true;
    io.WriteCommand(cmd);

    const ActuatorCommand out = io.ReadCommand(0.0);
    assert(out.effort == 0.0 && !out.enabled && "Disengaged: zero effort");
    assert(!io.IsEngaged());
}

// ---------------------------------------------------------------------------
// Engage + ramp monotonic
// ---------------------------------------------------------------------------
static void test_engage_ramp() {
    ActuatorIOPolicy policy;
    policy.watchdog_timeout_s = 100.0;
    policy.ramp_time_s        = 1.0;

    ChActuatorIO io(policy);

    ActuatorCommand cmd;
    cmd.effort  = 100.0;
    cmd.enabled = true;
    io.WriteCommand(cmd);

    io.Engage(0.0);

    double prev = -1.0;
    const double dt = 0.1;
    for (double t = 0.0; t <= 1.0 + 1e-9; t += dt) {
        const double e = io.ReadCommand(t).effort;
        assert(e >= prev - 1e-9 && "Ramp is monotonically non-decreasing");
        prev = e;
    }

    const double e_full = io.ReadCommand(1.5).effort;
    assert(e_full == 100.0 && "After ramp: full effort");
}

// ---------------------------------------------------------------------------
// Clamps: model-level and output
// ---------------------------------------------------------------------------
static void test_clamps() {
    ActuatorIOPolicy policy;
    policy.watchdog_timeout_s  = 100.0;
    policy.ramp_time_s         = 0.0;
    policy.model_effort_clamp  = 50.0;
    policy.output_effort_clamp = 30.0;

    ChActuatorIO io(policy);
    io.Engage(0.0);

    ActuatorCommand cmd;
    cmd.effort  = 200.0;
    cmd.enabled = true;
    io.WriteCommand(cmd);

    const ActuatorCommand after_model = io.ReadCommand(1.0);
    assert(after_model.effort <= 50.0 && "Model-level clamp applied");

    const double after_output = io.ApplyOutputClamp(after_model.effort);
    assert(after_output <= 30.0 && "Output clamp applied");
}

// ---------------------------------------------------------------------------
// Seqlock: concurrent writer/reader — reader never sees torn command
// ---------------------------------------------------------------------------
static void test_seqlock_concurrent() {
    ActuatorIOPolicy policy;
    policy.watchdog_timeout_s = 1000.0;

    ChActuatorIO io(policy);
    io.Engage(0.0);

    std::atomic<bool> stop{false};
    std::atomic<int>  torn_count{0};

    std::thread writer([&]() {
        int i = 0;
        while (!stop.load(std::memory_order_relaxed)) {
            ActuatorCommand c;
            c.effort  = (i % 2 == 0) ? 100.0 : 200.0;
            c.enabled = true;
            io.WriteCommand(c);
            ++i;
        }
    });

    for (int iter = 0; iter < 100000; ++iter) {
        ActuatorCommand c = io.ReadCommand(0.0);
        if (std::isnan(c.effort) || std::abs(c.effort) > 201.0) {
            ++torn_count;
        }
    }

    stop.store(true, std::memory_order_relaxed);
    writer.join();

    assert(torn_count == 0 && "Seqlock: no torn reads observed");
}

// ---------------------------------------------------------------------------
// Watchdog: stale command drives effort to zero
// ---------------------------------------------------------------------------
static void test_watchdog() {
    ActuatorIOPolicy policy;
    policy.watchdog_timeout_s = 0.05;
    policy.ramp_time_s        = 0.0;

    ChActuatorIO io(policy);
    io.Engage(0.0);

    ActuatorCommand cmd;
    cmd.effort  = 100.0;
    cmd.enabled = true;
    io.WriteCommand(cmd);

    const double e_fresh = io.ReadCommand(0.0).effort;
    assert(e_fresh == 100.0 && "Fresh command: full effort");

    std::this_thread::sleep_for(std::chrono::milliseconds(100));

    const double e_stale = io.ReadCommand(0.1).effort;
    assert(e_stale == 0.0 && "Stale command: watchdog zeros effort");
}

// ---------------------------------------------------------------------------
// Disengage: returning to zero
// ---------------------------------------------------------------------------
static void test_disengage() {
    ActuatorIOPolicy policy;
    policy.watchdog_timeout_s = 100.0;
    policy.ramp_time_s        = 0.0;

    ChActuatorIO io(policy);
    io.Engage(0.0);

    ActuatorCommand cmd;
    cmd.effort = 50.0;
    cmd.enabled = true;
    io.WriteCommand(cmd);

    assert(io.ReadCommand(0.0).effort == 50.0);

    io.Disengage();
    assert(io.ReadCommand(0.0).effort == 0.0 && "After disengage: zero effort");
}

int main() {
    test_engage_gate();
    test_engage_ramp();
    test_clamps();
    test_seqlock_concurrent();
    test_watchdog();
    test_disengage();
    return 0;
}
