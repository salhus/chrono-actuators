# chrono-actuators architecture

This document describes the architecture of `chrono-actuators` and the reasoning behind its design boundaries.

## 1. Problem statement

An actuator modelled as a prescribed force is an ideal effort source: it delivers commanded effort regardless of velocity, has no output impedance, cannot saturate, cannot be back-driven, and has no efficiency asymmetry between quadrants.  Wherever the actuator is the power conversion path — a PTO in a wave energy converter, a joint drive in a robot arm, a linear actuator in a test rig — that idealisation corrupts the quantity being computed and any control law tuned against it.

`chrono-actuators` replaces the ideal-effort-source pattern with actuator models that have real torque-speed characteristics, finite output impedance, and internal state.  The library is domain-neutral: the same models apply to marine PTO, robotic joints, and any other effort-producing actuator.  Wave energy converter PTO modelling (replacing WEC-Sim's prescribed-force and damping elements) is the first validation case; it is not the library's identity.

The implementation separates three concerns that should not be coupled:

1. **Physics** — what the actuator does, expressed in C++ stdlib with no dependency on any simulation engine.
2. **Chrono integration** — how the physics couples into a multibody simulation.
3. **Hardware edge** — how commands and telemetry flow between the simulation and an async source (device driver, ROS node, HIL controller).

---

## 2. Layering

```text
async command source / device / ROS / HIL
                |
                v
          ChActuatorIO
                |
                v
      frozen ActuatorCommand per step
                |
                v
        ActuatorModel interface
          |                |
          |                +--> stateful ODE path
          |                     via ChActuatorDynamics : ChExternalDynamicsODE
          |
          +--> zero-state algebraic path
                via TSDA / RSDA / motor-function adapters
```

### 2.1 Model layer (`models/`)

Files in `src/chrono_actuators/models/` are **Chrono-free** — they depend only on the C++ standard library.  The reason is concrete: the same model must run inside a Chrono simulation, inside a HIL loop against physical hardware, and inside unit tests with no multibody engine present.  A dependency on `ChSystem` would make the second and third impossible.

Model-layer files define:

- command/state PODs (`ActuatorCommand`, `ActuatorState`),
- the base model contract (`ActuatorModel`),
- telemetry and envelope helpers,
- concrete reusable actuator models.

This boundary is enforced structurally: `test_model_layer.cpp` is compiled without Chrono on the include path.

### 2.2 Hardware edge (`ChActuatorIO`)

`ChActuatorIO.h` is also Chrono-free.  It is the boundary between the simulation and any async command source — a hardware device driver, a ROS node, or a HIL controller.  The hot path is non-blocking: reads and writes use a seqlock over `std::atomic<uint32_t>` rather than a mutex or an `std::atomic<T>` over the full payload (atomics wider than the platform's lock-free width fall back to a global lock table).

`ChActuatorIO` owns:

- seqlock command and telemetry exchange,
- watchdog for bounded command staleness,
- engage/disengage gating,
- ramp-in on re-engage,
- independent clamp policy.

### 2.3 Chrono binding layer

Files in `src/chrono_actuators/chrono/` bind the neutral model API into concrete Chrono mechanisms:

- TSDA force functor,
- RSDA torque functor,
- motor-function adapter,
- shaft driveline adapter,
- monolithic ODE binding through `ChExternalDynamicsODE`.

---

## 3. Two evaluation contracts

The key architectural split is that not all actuator models should be evaluated the same way.

### 3.1 Contract A: zero-state algebraic models

These satisfy:

```text
GetNumStates() == 0
```

Examples:

- linear damper,
- spring-damper PTO,
- static saturation or friction laws.

They must expose a **pure** function:

```cpp
double ComputeEffort(const ActuatorCommand&, const ActuatorState&) const;
```

These models are safe to bind through Chrono force/torque functors because Chrono may re-query them multiple times at the same time and configuration.

### 3.2 Contract B: stateful ODE models

These satisfy:

```text
GetNumStates() > 0
```

Examples:

- electrical winding-current dynamics,
- hydraulic pressure-volume dynamics,
- actuator-internal valve or current-loop states.

They must additionally provide:

- initial conditions,
- RHS evaluation,
- optional analytic Jacobian,
- stiffness declaration,
- effort extraction from the integrated state.

These models should **not** be hidden behind an algebraic force functor. They must be integrated as part of the Chrono system through `ChActuatorDynamics`.

---

## 4. Three invariants

### 4.1 Invariant A — re-query purity

For zero-state models:

> `ComputeEffort(command, state)` must be a pure function of its arguments.

Why:

- implicit integrators may evaluate force functors multiple times per step,
- repeated queries at identical arguments must produce identical results,
- any hidden mutation would silently produce solver-dependent physics.

Consequences:

- no hidden integrators,
- no rate limiting in functors,
- no logging, counters, or mutable accumulators in zero-state `ComputeEffort`.

### 4.2 Invariant B — sign convention stability

The repository defines one canonical sign convention:

- positive linear effort **extends**,
- positive rotary effort acts in the **positive angle direction**.

Why:

- Chrono motor conventions already follow this pattern,
- model reuse becomes straightforward,
- application-specific sign inversions remain local instead of contaminating shared code.

### 4.3 Invariant C — frozen command per accepted step

The physics side reads the latest command once per accepted step, then uses that frozen snapshot during all model evaluations in the step.

Why:

- async writers must not race inside solver callbacks,
- command jitter should not depend on how many times a force functor is re-queried,
- this preserves deterministic solver behavior.

---

## 5. Envelope split: pure vs step-scoped

The effort envelope deliberately separates operations into two categories.

### 5.1 Pure operations

These are safe anywhere, including inside functors:

- deadband,
- friction as a pure function of velocity,
- quadrant efficiency,
- saturation.

### 5.2 Step-scoped operations

These are only valid once per accepted step:

- rate limiting.

Rate limiting stores previous-step information. If it were applied inside a force functor, re-queries within the same solver step would mutate state and violate Invariant A.

---

## 6. Mode taxonomy

`ChActuatorIO` defines a three-mode taxonomy for how state ownership is interpreted downstream.

| Mode | State owner | Chrono role | Typical use |
|---|---|---|---|
| `Sil` | Chrono | integrates actuator + plant, publishes state | software-in-loop |
| `Parallel` | hardware/controller | shadows state, may apply observer correction | parallel simulation / digital twin |
| `Hil` | hardware | evaluates load effort from measured kinematics | hardware-in-the-loop |

This taxonomy is descriptive, not invasive: it does not force middleware choices into the model layer.

---

## 7. Why the hardware edge uses a seqlock

The command and telemetry payloads are larger than a trivially lock-free native scalar.

Using `std::atomic<ActuatorCommand>` or `std::atomic<ActuatorTelemetry>` would typically fall back to runtime helper machinery or a mutex table. That is undesirable in the tight physics path.

Instead the module uses a classic seqlock pattern:

1. writer publishes an odd sequence number,
2. writer copies the payload,
3. writer publishes an even sequence number,
4. reader retries if it observed an odd or changed sequence.

Only `std::atomic<uint32_t>` is used atomically, which remains genuinely lock-free on mainstream targets.

The result is:

- no blocking in physics,
- consistent snapshots,
- simple testability.

---

## 8. Why use `ChExternalDynamicsODE`

Stateful actuator dynamics must be integrated with the multibody system, not in a disconnected side loop.

### 8.1 Benefits

Using `ChExternalDynamicsODE` gives:

- simultaneous integration with the main Chrono state,
- compatibility with implicit solvers for stiff problems,
- Jacobian assembly through `CalculateJac`,
- solver-managed step-size control,
- no duplicated integration hierarchy.

### 8.2 Why this matters for electric actuators

Electrical inductance can make winding-current dynamics stiff. Small `L` with moderate `R` yields fast internal time constants that are poorly served by naïve explicit sub-stepping when the surrounding mechanics are integrated implicitly.

### 8.3 Solver requirement for stiff ODE models

**Any `ChActuatorDynamics` instance whose model returns `IsStiff() == true` requires a direct solver and an implicit timestepper.**

When `IsStiff()` is true, `ChExternalDynamicsODE::InjectKRMMatrices()` inserts a KRM block into the system descriptor. The default `ChSystemNSC` PSOR solver is an iterative VI solver (complementarity-based) that **cannot** consume KRM blocks. Attempting to run will abort with:

```
ChSolverPSOR: Can NOT use PSOR solver if the system includes stiffness or damping matrices
```

Required setup before `sys.Add(dyn)`:

```cpp
#include "chrono/solver/ChDirectSolverLS.h"
#include "chrono/timestepper/ChTimestepperImplicit.h"

auto solver = chrono_types::make_shared<ChSolverSparseLU>();  // or ChSolverSparseQR
sys.SetSolver(solver);
solver->UseSparsityPatternLearner(true);
solver->LockSparsityPattern(true);
solver->SetVerbose(false);

sys.SetTimestepperType(ChTimestepper::Type::EULER_IMPLICIT);
auto integrator = std::static_pointer_cast<ChTimestepperEulerImplicit>(sys.GetTimestepper());
integrator->SetMaxIters(50);
integrator->SetAbsTolerances(1e-4, 1e2);
```

Both `ChSolverSparseLU` and `ChSolverSparseQR` are available in the base Chrono distribution (no optional modules required), declared in `chrono/solver/ChDirectSolverLS.h`.

Models with `GetNumStates() > 0` but `IsStiff() == false` do not trigger KRM injection and do not need this setup.

---

## 9. Sign conventions and downstream integration

### 9.1 Canonical convention

The repository uses:

- positive force = extension direction,
- positive torque = positive-angle direction.

### 9.2 PTO / extraction interpretation

Power-extracting devices like dampers often oppose motion, so they naturally produce:

- negative force for positive extension velocity,
- positive force for negative extension velocity.

That is not a contradiction; it is the intended physics under the canonical sign convention.

### 9.3 Downstream inversion rule

If a downstream system defines positive force differently, **invert at the downstream boundary only**.

Do **not** embed application-specific polarity inside shared model code.

This rule keeps:

- unit tests simple,
- cross-binding behavior consistent,
- documentation unambiguous.

---

## 10. Included models in the rewrite

### 10.1 `LinearDamperModel`

A zero-state PTO baseline:

```text
F = -B*v
```

### 10.2 `ReactiveDamperModel`

A zero-state reactive PTO:

```text
F = -K*x - B*v
```

### 10.3 `ElectricActuatorModel`

A one-state DC/BLDC actuator with:

- state `i` (winding current),
- RHS `(V - R*i - Ke*omega) / L`,
- DC bus voltage limit `V_bus` that gives the model a real torque-speed characteristic,
- gear ratio and efficiency,
- viscous and Coulomb friction,
- current and effort clamping,
- analytic Jacobian `-R/L`,
- `IsStiff() == true`.

#### Two operating regimes

**Unsaturated** (`|Ke * omega_motor| < V_bus`):

The back-EMF feedforward in `CommandToVoltage` exactly cancels the `- Ke * omega` term in the ODE RHS.  The commanded current — and therefore the commanded effort — is delivered with only the first-order winding-current lag.

**Saturated** (`Ke * omega_motor → V_bus`):

The terminal voltage is clamped at `V_bus`.  Current falls as:

```text
i = (V_bus - Ke * omega_motor) / R
```

This produces a real torque-speed droop and a finite no-load speed, making the model a two-port transducer with real output impedance.

#### Torque-speed endpoints

```text
tau_stall    = Kt * (V_bus / R) * N * eta          (omega = 0, full bus voltage)
omega_nl_out = V_bus / (Ke * N)                     (output shaft, zero load torque)
```

where `N = gear_ratio = omega_motor / omega_output`.

These are exposed as `GetStallEffort()` and `GetNoLoadSpeed()`.

---

## 11. Binding selection guide

| Model type | Preferred binding | Reason |
|---|---|---|
| zero-state translational law | `ChActuatorFunctorTSDA` | safe repeated force evaluation |
| zero-state rotary law | `ChActuatorFunctorRSDA` | safe repeated torque evaluation |
| zero-state motor-link law | `ChActuatorFunction` | integrates with Chrono motor-function API |
| shaft driveline | `ChActuatorShaft` | direct 1-D driveline topology |
| stateful stiff or non-stiff ODE | `ChActuatorDynamics` | monolithic integration |

---

## 12. Testing strategy

The rewrite intentionally creates three test bands:

1. **Chrono-free model tests**
   - validate invariants, envelope, and electric model math.

2. **Chrono-free I/O tests**
   - validate seqlock correctness and safety policy.

3. **Chrono-backed binding tests**
   - validate that binding-layer sign and re-query semantics match the model contract.

This split catches architecture regressions early without needing a full Chrono runtime for every check.

---

## 13. Extension guidance

Future contributors should follow these rules:

### 13.1 Adding a new zero-state model

- keep it Chrono-free,
- keep `ComputeEffort` pure,
- test it in `test_model_layer.cpp` or equivalent.

### 13.2 Adding a new stateful model

- implement the ODE hooks in `ActuatorModel`,
- declare stiffness honestly,
- provide an analytic Jacobian when practical,
- prefer `ChActuatorDynamics` over hidden explicit sub-stepping.

### 13.3 Adding middleware integration

- stop at `ChActuatorIO`,
- never block the physics thread,
- never mutate command state from inside Chrono callbacks.

---

## 14. Summary

The rewrite is built around a small set of durable ideas:

- two explicit evaluation contracts,
- three invariants,
- one canonical sign convention,
- a non-blocking hardware edge,
- and monolithic Chrono integration for stateful actuator internals.

That structure is what makes the module suitable for simulation, HIL, and downstream actuator reuse instead of remaining a scaffold.
