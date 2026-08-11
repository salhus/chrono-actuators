# chrono-actuators

`chrono-actuators` is a standalone external Chrono module scaffold for actuator-augmented multibody simulation, designed for portability, performance-conscious interfaces, and ROS/HIL compatibility.

## Purpose

- Provide a general actuator-modeling layer for MBD and sim-to-real workflows.
- Keep model dynamics engine-neutral (pure C++ types).
- Use a Chrono adapter boundary to bind effort outputs to Chrono motors/shafts.
- Support async I/O (ROS/HIL) without blocking synchronous physics steps.

The first intended application is WEC PTO modeling, but architecture is domain-neutral.

## Architecture (scaffold v0.1)

- **`actuators_sim` (implemented as `src/chrono_actuators`)**
  - `models/`: engine-neutral interfaces and model declarations (no Chrono/ROS includes)
  - `ChActuatorAdapter`: Chrono boundary for marshalling state/effort
  - `ChActuator`: `SetCommand` / `GetState` / `Advance` contract
  - `CommandStateCache`: non-blocking async/sync handoff stub
- **`actuators_io/ros`** (optional, OFF by default): placeholder for ros2_control-legible transport.
- **`actuators_io/hil`** (optional, OFF by default): placeholder for hardware-in-the-loop transport.

```text
Setpoint/Command source (async ROS/HIL) --> CommandStateCache --> ChActuator::Advance(dt)
                                                             |-> model::ActuatorModel (Chrono-free)
                                                             |-> ChActuatorAdapter (Chrono-aware)
                                                             '--> state publish back to cache
```

## Build

Prerequisite: installed Project Chrono and `Chrono_DIR` available.

```bash
cmake -S . -B build -DChrono_DIR=/path/to/chrono/lib/cmake/Chrono
cmake --build build -j
ctest --test-dir build --output-on-failure
```

### Options

- `CHRONO_ACTUATORS_ENABLE_ROS` (default `OFF`)
- `CHRONO_ACTUATORS_ENABLE_HIL` (default `OFF`)
- `CHRONO_ACTUATORS_BUILD_DEMOS` (default `ON`)
- `CHRONO_ACTUATORS_BUILD_TESTS` (default `ON`)

## Roadmap

- **v0.1**: scaffold (build system + interfaces + docs + placeholders)
- **v0.2**: geared-electric actuator model + first physics demo
- **v0.3**: ROS transport integration
- **v0.4**: HIL transport integration

## Contribution / upstream note

The project follows Chrono optional-module conventions to stay upstreamable as a future Chrono module contribution.
