# chrono-actuators architecture (scaffold reference)

This document captures the architectural invariants for the scaffold-only v0.1 PR.

## Invariants

1. **Layered dependency split**
   - `actuators_sim` (`src/chrono_actuators`) contains model interfaces, Chrono adapter, and command/state contract.
   - Model headers in `models/` depend only on the C++ standard library.
   - ROS/HIL are optional sibling transports, OFF by default.

2. **Engine-neutral model + Chrono-woven adapter**
   - Model API: `Effort ComputeEffort(command, state, dt)` over plain structs.
   - `ChActuatorAdapter` is the single Chrono touchpoint.

3. **Effort-only actuator output**
   - Actuators output effort (force/torque) only.
   - Position/velocity intents belong to optional controller logic (`ControllerBase` contract only).

4. **Quasi-static electrical default + optional sub-stepper seam**
   - `ActuatorModel::AdvanceInternalState(dt)` provides optional fixed-step internal integration seam.
   - Default implementation is no-op for cheap quasi-static evaluation.

5. **Async comms, sync physics, non-blocking cache contract**
   - `CommandStateCache` defines non-blocking command/state exchange.
   - Physics path is never blocked by ROS/HIL transport threads.

6. **ros2_control legibility (future layer)**
   - ROS layer is documented to mirror `SystemInterface`/controller split and standard message usage.

7. **Scaffold-only scope**
   - No actuator physics/model bodies in v0.1.
   - TODO markers (`TODO(v0.2)`) identify planned implementation points.

## Priority alignment

- **Performance**: lock-free/non-blocking command-state handoff and explicit physics-step contract.
- **Portability**: Chrono-free model layer + C++-only interfaces.
- **ROS compatibility**: dedicated optional ROS transport seam preserving deterministic sim stepping.
