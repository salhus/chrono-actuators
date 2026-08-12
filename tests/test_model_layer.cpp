// =============================================================================
// Test: model layer — no Chrono dependency.
// Covers: Invariant A (re-query purity), Envelope, ElectricActuatorModel
//         analytic steady state, sign convention.
// =============================================================================

#include <cassert>
#include <cmath>
#include <cstring>
#include <stdexcept>

#include "chrono_actuators/models/ActuatorModel.h"
#include "chrono_actuators/models/Telemetry.h"
#include "chrono_actuators/models/Envelope.h"
#include "chrono_actuators/models/LinearDamperModel.h"
#include "chrono_actuators/models/ReactiveDamperModel.h"
#include "chrono_actuators/models/ElectricActuatorModel.h"

using namespace chrono::actuators;

// ---------------------------------------------------------------------------
// Invariant A: zero-state model is a pure function — bit-identical on re-query
// ---------------------------------------------------------------------------
static void test_requery_purity() {
    LinearDamperModel damper(500.0);

    ActuatorCommand cmd;
    cmd.effort  = 100.0;
    cmd.enabled = true;

    ActuatorState state;
    state.displacement = 0.3;
    state.velocity     = 0.7;
    state.time         = 1.234;

    const double e1 = damper.ComputeEffort(cmd, state);
    const double e2 = damper.ComputeEffort(cmd, state);
    const double e3 = damper.ComputeEffort(cmd, state);

    assert(e1 == e2 && e2 == e3 && "Invariant A: re-query must return bit-identical result");
}

// ---------------------------------------------------------------------------
// LinearDamperModel: F = -B*v, sign convention
// ---------------------------------------------------------------------------
static void test_linear_damper_sign() {
    LinearDamperModel damper(200.0);

    ActuatorState s;
    s.velocity = 1.0;

    ActuatorCommand cmd;

    const double f = damper.ComputeEffort(cmd, s);
    assert(f == -200.0 && "LinearDamperModel: F = -B*v, positive v → negative F");
    assert(damper.GetNumStates() == 0);
}

// ---------------------------------------------------------------------------
// ReactiveDamperModel: F = -K*x - B*v
// ---------------------------------------------------------------------------
static void test_reactive_damper() {
    ReactiveDamperModel::Params p;
    p.stiffness = 1000.0;
    p.damping   = 100.0;
    ReactiveDamperModel model(p);

    ActuatorState s;
    s.displacement = 0.1;
    s.velocity     = 0.5;

    ActuatorCommand cmd;
    const double f = model.ComputeEffort(cmd, s);
    assert(std::abs(f - (-1000.0 * 0.1 - 100.0 * 0.5)) < 1e-10);
    assert(model.GetNumStates() == 0);
}

// ---------------------------------------------------------------------------
// Envelope: saturation
// ---------------------------------------------------------------------------
static void test_envelope_saturation() {
    EnvelopeParams ep;
    ep.saturation.max_extend  =  50.0;
    ep.saturation.max_retract = -30.0;

    ActuatorTelemetry t{};
    double e;

    e = ApplyPureEnvelope(100.0, 1.0, ep, t);
    assert(e == 50.0 && t.effort_saturated);

    t = {};
    e = ApplyPureEnvelope(-60.0, -1.0, ep, t);
    assert(e == -30.0 && t.effort_saturated);

    t = {};
    e = ApplyPureEnvelope(20.0, 1.0, ep, t);
    assert(e == 20.0 && !t.effort_saturated);
}

// ---------------------------------------------------------------------------
// Envelope: deadband
// ---------------------------------------------------------------------------
static void test_envelope_deadband() {
    EnvelopeParams ep;
    ep.deadband = 5.0;

    ActuatorTelemetry t{};
    assert(ApplyPureEnvelope(3.0, 0.0, ep, t) == 0.0);
    t = {};
    assert(ApplyPureEnvelope(6.0, 1.0, ep, t) != 0.0);
}

// ---------------------------------------------------------------------------
// Envelope: quadrant efficiency
// ---------------------------------------------------------------------------
static void test_envelope_quadrant_efficiency() {
    EnvelopeParams ep;
    ep.efficiency.motoring   = 0.9;
    ep.efficiency.generating = 0.8;

    ActuatorTelemetry t{};

    const double e_mot = ApplyPureEnvelope(100.0, 1.0, ep, t);
    assert(std::abs(e_mot - 90.0) < 1e-10 && "Motoring efficiency");

    t = {};
    const double e_gen = ApplyPureEnvelope(-100.0, 1.0, ep, t);
    assert(std::abs(e_gen - (-80.0)) < 1e-10 && "Generating efficiency");
}

// ---------------------------------------------------------------------------
// Rate limiter (step-scoped)
// ---------------------------------------------------------------------------
static void test_rate_limiter() {
    EnvelopeParams ep;
    ep.max_rate = 100.0;

    double prev = 0.0;
    ActuatorTelemetry t{};

    double e = ApplyRateLimit(200.0, prev, 0.01, ep.max_rate, t);
    assert(e == 1.0 && t.rate_limited && "Rate limiting applied");
    assert(prev == 1.0 && "prev updated");

    t = {};
    e = ApplyRateLimit(1.5, prev, 0.01, ep.max_rate, t);
    assert(e == 1.5 && !t.rate_limited);
}

// ---------------------------------------------------------------------------
// ElectricActuatorModel: steady-state torque = Kt*(V-Ke*omega)/R
// ---------------------------------------------------------------------------
static void test_electric_steady_state() {
    ElectricActuatorParams p;
    p.R  = 2.0;
    p.L  = 1e-4;
    p.Kt = 0.5;
    p.Ke = 0.5;
    p.gear_ratio      = 10.0;
    p.gear_efficiency = 1.0;

    ElectricActuatorModel model(p);
    assert(model.GetNumStates() == 1);
    assert(model.IsStiff());

    ActuatorCommand cmd;
    cmd.effort  = 3.0;
    cmd.enabled = true;

    ActuatorState state;
    state.velocity = 0.0;

    const double tau = model.ComputeEffort(cmd, state);
    assert(std::abs(tau - 3.0) < 1e-6 && "Stall torque matches commanded effort");
}

// ---------------------------------------------------------------------------
// ElectricActuatorModel: analytic Jacobian vs finite difference
// ---------------------------------------------------------------------------
static void test_electric_jacobian() {
    ElectricActuatorParams p;
    p.R  = 1.5;
    p.L  = 2e-3;
    p.Kt = 0.3;
    p.Ke = 0.3;
    p.gear_ratio      = 5.0;
    p.gear_efficiency = 0.95;

    ElectricActuatorModel model(p);

    ActuatorCommand cmd;
    cmd.effort = 1.0;
    ActuatorState state;
    state.velocity = 0.2;

    const double y[1] = {0.5};
    double jac_analytic[1];
    const bool ok = model.CalculateJac(0.0, y, state, cmd, jac_analytic);
    assert(ok && "CalculateJac must return true for ElectricActuatorModel");

    const double expected = -p.R / p.L;
    assert(std::abs(jac_analytic[0] - expected) < 1e-10 && "Analytic Jacobian = -R/L");

    const double h = 1e-6;
    const double y_h[1] = {y[0] + h};
    double rhs0[1], rhsh[1];
    model.CalculateRHS(0.0, y,   state, cmd, rhs0);
    model.CalculateRHS(0.0, y_h, state, cmd, rhsh);
    const double jac_fd = (rhsh[0] - rhs0[0]) / h;
    assert(std::abs(jac_analytic[0] - jac_fd) < 1e-4 && "Analytic matches FD Jacobian");
}

// ---------------------------------------------------------------------------
// ElectricActuatorModel: no-load speed
// ---------------------------------------------------------------------------
static void test_electric_no_load_speed() {
    ElectricActuatorParams p;
    p.R  = 1.0;
    p.L  = 1e-4;
    p.Kt = 0.1;
    p.Ke = 0.1;
    p.gear_ratio      = 1.0;
    p.gear_efficiency = 1.0;

    ElectricActuatorModel model(p);
    ActuatorCommand cmd;
    cmd.effort = 0.0;
    ActuatorState state;
    state.velocity = 10.0;

    const double tau = model.ComputeEffort(cmd, state);
    assert(std::abs(tau) < 1e-6 && "No-load: zero commanded effort yields near-zero torque");
}

// ---------------------------------------------------------------------------
// Sign convention: functor sign is positive = extend
// ---------------------------------------------------------------------------
static void test_sign_convention_linear_damper() {
    LinearDamperModel damper(100.0);
    ActuatorState s;
    s.velocity = 2.0;
    ActuatorCommand cmd;
    const double f = damper.ComputeEffort(cmd, s);
    assert(f < 0.0 && "Extension velocity produces negative force (energy extraction)");
}

int main() {
    test_requery_purity();
    test_linear_damper_sign();
    test_reactive_damper();
    test_envelope_saturation();
    test_envelope_deadband();
    test_envelope_quadrant_efficiency();
    test_rate_limiter();
    test_electric_steady_state();
    test_electric_jacobian();
    test_electric_no_load_speed();
    test_sign_convention_linear_damper();
    return 0;
}
