# chrono-actuators ROS I/O scaffold (placeholder)

This directory is intentionally scaffold-only for v0.1.

## Intended design

- Mirror `ros2_control` legibility: hardware `SystemInterface`-like transport split from controller logic.
- Use standard ROS 2 message flows (`sensor_msgs/JointState`, `control_msgs` family).
- Bridge commands/states through `CommandStateCache` with asynchronous ROS threads.
- Preserve synchronous physics stepping: ROS paths must never block `ChActuator::Advance`.

No ROS code is shipped in this PR by design.
