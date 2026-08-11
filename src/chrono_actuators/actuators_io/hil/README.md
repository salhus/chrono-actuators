# chrono-actuators HIL I/O scaffold (placeholder)

This directory is intentionally scaffold-only for v0.1.

## Intended design

- Hardware-facing transport for hard-real-time integration (e.g., EtherCAT, analog I/O, drive interfaces).
- Explicitly distinct from ROS paths: HIL timing and fault boundaries are independent.
- Uses the same asynchronous command/state cache contract so hardware I/O never blocks physics stepping.

No hardware SDK code is shipped in this PR by design.
