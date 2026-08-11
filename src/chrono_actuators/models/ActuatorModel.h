#ifndef CHRONO_ACTUATORS_MODEL_ACTUATOR_MODEL_H
#define CHRONO_ACTUATORS_MODEL_ACTUATOR_MODEL_H

#include <cstdint>

namespace chrono {
namespace actuators {
namespace model {

/**
 * @brief Scalar effort type used by actuator models.
 *
 * Effort always means force (linear actuator) or torque (rotary actuator).
 */
using Effort = double;

/**
 * @brief Engine-neutral actuator state visible to model code.
 *
 * This POD intentionally carries no Chrono or middleware types so that model
 * implementations remain portable across simulation engines and HIL backends.
 */
struct ActuatorState {
    double position = 0.0;          ///< Joint/shaft position in SI units.
    double velocity = 0.0;          ///< Joint/shaft velocity in SI units.
    double measured_effort = 0.0;   ///< Last applied/measured effort in SI units.
    double time = 0.0;              ///< Monotonic simulation timestamp [s].
};

/**
 * @brief Command mode tag.
 *
 * Actuators are effort-output devices. Position/velocity setpoints are optional
 * upstream hints for an external controller layer and are never interpreted as
 * direct position constraints by the model itself.
 */
enum class CommandMode : std::uint8_t {
    kEffort = 0,
    kPositionHint = 1,
    kVelocityHint = 2
};

/**
 * @brief Engine-neutral command payload sent into actuator models.
 */
struct ActuatorCommand {
    double effort_setpoint = 0.0;        ///< Desired effort command in SI units.
    bool has_position_setpoint = false;  ///< Whether position_setpoint is populated.
    double position_setpoint = 0.0;      ///< Optional controller-facing position hint.
    bool has_velocity_setpoint = false;  ///< Whether velocity_setpoint is populated.
    double velocity_setpoint = 0.0;      ///< Optional controller-facing velocity hint.
    CommandMode mode = CommandMode::kEffort;  ///< Active setpoint semantic.
};

/**
 * @brief Base interface for all engine-neutral actuator models.
 *
 * Architectural contract:
 * - Model layer depends only on the C++ standard library.
 * - No Chrono, ROS, or hardware headers are allowed here.
 * - ComputeEffort must be deterministic for a given command/state/dt.
 *
 * Numerical seam:
 * - Default expectation is cheap quasi-static/algebraic effort evaluation.
 * - Models with stiff internal electrical/hydraulic dynamics may override
 *   AdvanceInternalState() to run an internal fixed-step sub-integrator
 *   (e.g., RK) before/after ComputeEffort.
 */
class ActuatorModel {
  public:
    virtual ~ActuatorModel() = default;

    /**
     * @brief Compute output effort from command/state for the current step.
     *
     * @param command Latest command snapshot.
     * @param state Current actuator/joint state snapshot.
     * @param dt Physics step size [s].
     * @return Effort command to apply at the plant interface.
     */
    virtual Effort ComputeEffort(const ActuatorCommand& command, const ActuatorState& state, double dt) = 0;

    /**
     * @brief Optional internal-state advance seam.
     *
     * Default implementation is a no-op to preserve the quasi-static baseline.
     * Override in v0.2+ models that require fixed-step internal integration.
     */
    virtual void AdvanceInternalState(double dt) {
        (void)dt;
    }
};

}  // namespace model
}  // namespace actuators
}  // namespace chrono

#endif
