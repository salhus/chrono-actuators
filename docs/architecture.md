# chrono-actuators architecture

This document describes the rewritten architecture of `chrono-actuators` and the reasoning behind its API boundaries.

## 1. Problem statement

Actuator models sit at the intersection of three concerns:

1. **physics-layer integration** with Chrono,
2. **model-layer portability** for controls, HIL, and unit testing, and
3. **I/O-layer safety** for asynchronous command sources.

The rewrite separates those concerns explicitly so each can be verified independently.

### What this module adds over existing Chrono actuation primitives

Chrono's standard actuation primitives (`ChLinkMotorLinearForce`, `ChShaftsMotorLoad`, `ChLinkTSDA` force functors) are **ideal effort sources with zero output impedance**: they impose a force or torque and accept whatever velocity the multibody solve produces.  The load cannot push back through them and there is no torque-speed droop.

`ChHydraulicActuator` is the existing exception — a genuine multiport model with internal pressure-volume state and real impedance.

`ElectricActuatorModel` is its electromechanical counterpart.  The DC bus voltage limit `V_bus` is the concrete mechanism: once `Ke·ω_motor` consumes the available bus headroom, terminal voltage saturates, current falls, and effort droops to zero at the no-load speed.  The actuator cannot deliver infinite power at arbitrary speed — it is a two-port transducer, not an ideal effort source.  This symmetry with `ChHydraulicActuator` is the module's upstream argument.

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

### 2.1 Model layer

Files in `src/chrono_actuators/models/` are **Chrono-free**. They depend only on the standard library and define:

- command/state PODs,
- the base model contract,
- telemetry and envelope helpers,
- concrete reusable actuator models.

### 2.2 Hardware edge

`ChActuatorIO.h` is also Chrono-free. It owns:

- seqlock command and telemetry exchange,
- watchdog logic,
- engage/disengage gating,
- ramp-in,
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

### 8.4 Why mirror `ChHydraulicActuator`

Chrono already contains a successful pattern for monolithically integrated actuator internals in `ChHydraulicActuator`. Matching that structure:

- aligns with Chrono conventions,
- reduces conceptual surprise for Chrono developers,
- gives a path for future hydraulic and electro-hydraulic expansion.

---

## 9. Why keep the module in-process

The rewrite intentionally keeps the default execution model in-process instead of turning every actuator evaluation into an RPC or co-simulation call.

Reasons:

1. **solver semantics** — functors and ODE callbacks must be callable many times per step with low latency;
2. **determinism** — in-process evaluation avoids scheduling and transport jitter in the core loop;
3. **Jacobian participation** — stateful models need direct access to Chrono's monolithic integration path;
4. **portability** — the model layer remains engine-neutral without becoming transport-dependent.

External transports still exist, but they stop at the `ChActuatorIO` boundary.

---

## 10. Sign conventions and downstream integration

### 10.1 Canonical convention

The repository uses:

- positive force = extension direction,
- positive torque = positive-angle direction.

### 10.2 PTO / extraction interpretation

Power-extracting devices like dampers often oppose motion, so they naturally produce:

- negative force for positive extension velocity,
- positive force for negative extension velocity.

That is not a contradiction; it is the intended physics under the canonical sign convention.

### 10.3 Downstream inversion rule

If a downstream system defines positive force differently, **invert at the downstream boundary only**.

Do **not** embed application-specific polarity inside shared model code.

This rule keeps:

- unit tests simple,
- cross-binding behavior consistent,
- documentation unambiguous.

---

## 11. Included models in the rewrite

### 11.1 `LinearDamperModel`

A zero-state PTO baseline:

```text
F = -B*v
```

### 11.2 `ReactiveDamperModel`

A zero-state reactive PTO:

```text
F = -K*x - B*v
```

### 11.3 `ElectricActuatorModel`

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

#### Relationship to `ChHydraulicActuator`

`ChHydraulicActuator` is the existing Chrono class that already has genuine two-port (multiport) behavior with internal pressure-volume state.  `ElectricActuatorModel` is its electromechanical counterpart: both integrate actuator internal state simultaneously with the multibody system and both have a natural finite impedance.  The other Chrono actuation primitives (`ChLinkMotorLinearForce`, `ChShaftsMotorLoad`, `ChLinkTSDA` force functors) are ideal effort sources with zero output impedance — they impose force at any speed without droop.  This distinction is the module's upstream argument.

---

## 12. Binding selection guide

| Model type | Preferred binding | Reason |
|---|---|---|
| zero-state translational law | `ChActuatorFunctorTSDA` | safe repeated force evaluation |
| zero-state rotary law | `ChActuatorFunctorRSDA` | safe repeated torque evaluation |
| zero-state motor-link law | `ChActuatorFunction` | integrates with Chrono motor-function API |
| shaft driveline | `ChActuatorShaft` | direct 1-D driveline topology |
| stateful stiff or non-stiff ODE | `ChActuatorDynamics` | monolithic integration |

---

## 13. Testing strategy

The rewrite intentionally creates three test bands:

1. **Chrono-free model tests**
   - validate invariants, envelope, and electric model math.

2. **Chrono-free I/O tests**
   - validate seqlock correctness and safety policy.

3. **Chrono-backed binding tests**
   - validate that binding-layer sign and re-query semantics match the model contract.

This split catches architecture regressions early without needing a full Chrono runtime for every check.

---

## 14. Extension guidance

Future contributors should follow these rules:

### 14.1 Adding a new zero-state model

- keep it Chrono-free,
- keep `ComputeEffort` pure,
- test it in `test_model_layer.cpp` or equivalent.

### 14.2 Adding a new stateful model

- implement the ODE hooks in `ActuatorModel`,
- declare stiffness honestly,
- provide an analytic Jacobian when practical,
- prefer `ChActuatorDynamics` over hidden explicit sub-stepping.

### 14.3 Adding middleware integration

- stop at `ChActuatorIO`,
- never block the physics thread,
- never mutate command state from inside Chrono callbacks.

---

## 15. Summary

The rewrite is built around a small set of durable ideas:

- two explicit evaluation contracts,
- three invariants,
- one canonical sign convention,
- a non-blocking hardware edge,
- and monolithic Chrono integration for stateful actuator internals.

That structure is what makes the module suitable for simulation, HIL, and downstream actuator reuse instead of remaining a scaffold.
