# chrono-actuators

`chrono-actuators` is a full actuator module rewrite for Project Chrono that cleanly separates:

- a **Chrono-free model layer** for actuator physics and power conversion,
- a **hardware/HIL edge** for non-blocking command and telemetry exchange, and
- a **Chrono binding layer** for TSDA, RSDA, motor-function, shaft, and monolithic ODE integration paths.

The module is aimed at simulation, controls prototyping, HIL, and downstream PTO / driveline integration where sign discipline, re-query safety, and stiff internal dynamics matter.

## Design goals

- **Chrono-free model code** for portability and unit testing.
- **Explicit sign conventions** at every boundary.
- **Safe zero-state bindings** for algebraic models repeatedly queried by implicit solvers.
- **Monolithic integration** for stateful electrical or hydraulic dynamics.
- **Deterministic async I/O** using a seqlock-based command exchange instead of blocking mutex paths.
- **Thin downstream shims** instead of embedding application-specific polarity inversions or middleware assumptions.

## Repository layout

```text
src/chrono_actuators/
├── models/
│   ├── ActuatorModel.h            # Chrono-free base interface
│   ├── Telemetry.h                # per-step telemetry POD
│   ├── Envelope.h                 # pure and step-scoped effort shaping
│   ├── LinearDamperModel.h        # zero-state PTO baseline
│   ├── ReactiveDamperModel.h      # spring + damper PTO model
│   └── ElectricActuatorModel.h    # 1-state stiff DC/BLDC actuator
├── ChActuatorIO.h                 # seqlock + watchdog + engage gate + clamps
└── chrono/
    ├── ChActuatorFunctorTSDA.h    # zero-state TSDA binding
    ├── ChActuatorFunctorRSDA.h    # zero-state RSDA binding
    ├── ChActuatorFunction.h       # zero-state motor-function binding
    ├── ChActuatorShaft.h          # shaft driveline binding
    ├── ChActuatorDynamics.h/.cpp  # ChExternalDynamicsODE binding for stateful models

demos/
├── demo_ACT_linear_damper.cpp
├── demo_ACT_electric.cpp
├── demo_ACT_shaft.cpp
└── demo_ACT_hil_modes.cpp

tests/
├── test_model_layer.cpp
├── test_actuator_io.cpp
└── test_chrono_binding.cpp
```

## Architecture summary

### Two evaluation contracts

`ActuatorModel` supports two distinct contracts:

1. **Algebraic / zero-state models** (`GetNumStates() == 0`)
   - Must implement a pure `ComputeEffort(command, state) const`.
   - Safe for repeated force/torque functor re-queries at identical `(t, q, v)`.
   - Bind through `ChActuatorFunctorTSDA`, `ChActuatorFunctorRSDA`, or `ChActuatorFunction`.

2. **Stateful ODE models** (`GetNumStates() > 0`)
   - Provide initial conditions, RHS, optional analytic Jacobian, stiffness declaration, and `EffortFromStates`.
   - Bind through `ChActuatorDynamics`, which integrates internal states simultaneously with the Chrono system.

### Three invariants

1. **Invariant A — re-query purity**
   - Zero-state `ComputeEffort` must be a pure function.
   - No hidden accumulation, mutation, I/O, or time-history dependence.

2. **Invariant B — sign convention stability**
   - Positive effort means **extend** for linear actuators and **positive-angle rotation** for rotary actuators.
   - Any consumer with opposite sign semantics must invert in a local shim.

3. **Invariant C — frozen command per step**
   - The command sampled at the start of an accepted step is constant for all evaluations inside that step.
   - Async writers update command memory independently; physics reads once and freezes.

See `docs/architecture.md` for the full rationale and downstream guidance.

## Sign conventions

The repository uses a single sign convention throughout the model and binding layers:

- **Linear**: positive effort acts to **extend** the actuator.
- **Rotary**: positive effort acts in the **positive angular direction**.
- **Damper / PTO extraction**: when velocity is positive, resisting effort is negative.

Examples:

- `LinearDamperModel`: `F = -B*v`
- `ReactiveDamperModel`: `F = -K*x - B*v`
- `ElectricActuatorModel`: positive commanded effort maps to positive output torque

This matches Chrono motor conventions and keeps sign inversions out of the reusable core.

## Model layer

### `ActuatorModel`

The base interface is pure C++17/20 and intentionally contains **no Chrono headers**. It defines:

- `ActuatorCommand`
- `ActuatorState`
- `ComputeEffort(...) const`
- optional ODE hooks for stateful models

### `ActuatorTelemetry`

Telemetry is produced after effort computation and can record:

- applied effort,
- mechanical power,
- electrical power when modeled,
- effective efficiency,
- saturation and rate-limit flags.

### `Envelope`

The envelope layer explicitly separates:

- **Pure, re-query-safe operations**: deadband, friction, quadrant efficiency, saturation.
- **Step-scoped operations**: rate limiting.

That split is important: rate limiting must never be applied inside a Chrono force functor because force functors can be queried multiple times at the same simulation time.

### Included models

#### `LinearDamperModel`

A zero-state viscous damper for PTO baselines:

```text
F = -B*v
```

#### `ReactiveDamperModel`

A zero-state spring + damper model:

```text
F = -K*x - B*v
```

#### `ElectricActuatorModel`

A one-state DC/BLDC model with winding current as its internal state:

```text
y[0] = i

di/dt = (V - R*i - Ke*omega) / L
```

Key features:

- stiff declaration via `IsStiff() == true`,
- analytic Jacobian `d(rhs)/d(y) = -R/L`,
- output torque including gear efficiency and output friction,
- current and effort clamping,
- quasi-static `ComputeEffort` path plus full ODE path.

## Hardware/HIL edge

`ChActuatorIO` is Chrono-free and intended for ROS threads, device adapters, or synthetic HIL loops.

Features:

- **seqlock command exchange** using `std::atomic<uint32_t>` only,
- **staleness watchdog**,
- **explicit engage / disengage gate**,
- **linear ramp-in**,
- **model-level clamp** and **independent output clamp**,
- **zero command on destruction**,
- matching telemetry seqlock for readback.

The command exchange is non-blocking and avoids `std::atomic<ActuatorCommand>`, which would not be lock-free for this payload size on typical platforms.

## Chrono binding layer

### Zero-state bindings

- `ChActuatorFunctorTSDA`
- `ChActuatorFunctorRSDA`
- `ChActuatorFunction`

These are for algebraic models only. They depend on **Invariant A** being satisfied.

### Stateful binding

`ChActuatorDynamics` wraps a stateful model in `ChExternalDynamicsODE`, so Chrono integrates actuator internal states together with the multibody system. This is the preferred path for stiff electrical or hydraulic models because it gives:

- consistent implicit integration,
- Jacobian participation,
- solver-controlled step sizes,
- no external sub-stepping split.

### Shaft binding

`ChActuatorShaft` applies model output to a `ChShaftsMotorTorque` link for driveline and 1-D shaft topologies.

## Build

### Requirements

- CMake 3.18+
- a C++17 compiler (C++20 preferred when available)
- installed Project Chrono with `ChronoConfig.cmake`

### Configure, build, test

```bash
cmake -S . -B build -DChrono_DIR=/path/to/chrono/lib/cmake/Chrono
cmake --build build -j
ctest --test-dir build --output-on-failure
```

If `CMAKE_BUILD_TYPE` is not set, the module defaults to `Release`.

### CMake options

- `CHRONO_ACTUATORS_ENABLE_ROS` — optionally look for `chrono_ros`
- `CHRONO_ACTUATORS_ENABLE_HIL` — exported in config headers for downstream use
- `CHRONO_ACTUATORS_BUILD_DEMOS` — build demos
- `CHRONO_ACTUATORS_BUILD_TESTS` — build tests

## Tests

### `test_model_layer`

Chrono-free unit coverage for:

- re-query purity,
- sign convention,
- envelope behavior,
- electric steady state,
- analytic Jacobian correctness.

### `test_actuator_io`

Chrono-free unit coverage for:

- seqlock behavior,
- engage gate,
- ramp monotonicity,
- watchdog,
- layered clamps,
- disengage behavior.

### `test_chrono_binding`

Chrono-backed integration coverage for TSDA functor:

- identical re-query behavior,
- positive-velocity / negative-damper-force sign convention.

## Demos

- `demo_ACT_linear_damper` — PTO damper on a TSDA with absorbed power reporting.
- `demo_ACT_electric` — stateful electric actuator integrated through `ChActuatorDynamics`.
- `demo_ACT_shaft` — linear damper model applied to a shaft driveline via `ChActuatorShaft`.
- `demo_ACT_hil_modes` — synthetic command source exercising watchdog, engage gate, ramp, and clamps without ROS.

## Downstream integration guidance

### SEA-Stack / PTO consumers

If a downstream component defines positive force as **opposing extension** instead of **causing extension**, apply a local negation shim there. Do not fork or mutate the shared model sign convention.

### ROS or hardware transport

Use `ChActuatorIO` as the exchange boundary:

- writer thread publishes `ActuatorCommand`,
- physics thread calls `ReadCommand(sim_time)` once per accepted step,
- physics thread writes `ActuatorTelemetry`,
- transport thread polls `ReadTelemetry()` as needed.

### When to use each binding

| Situation | Binding |
|---|---|
| pure force law on TSDA | `ChActuatorFunctorTSDA` |
| pure torque law on RSDA | `ChActuatorFunctorRSDA` |
| force/torque motor link needing time-function interface | `ChActuatorFunction` |
| 1-D driveline / shaft train | `ChActuatorShaft` |
| internal ODE states, especially stiff | `ChActuatorDynamics` |

## Why this rewrite exists

The previous scaffold mixed placeholder interfaces, adapter stubs, and optional transport scaffolding. The rewrite makes the execution model explicit:

- pure model contracts are testable without Chrono,
- hardware safety policy is separable from physics,
- Chrono functor bindings are safe under implicit solver re-queries,
- stateful actuator dynamics use the same monolithic ODE pattern as Chrono hydraulic actuators.

## License

This repository is distributed under the BSD 3-Clause license in `LICENSE`.
