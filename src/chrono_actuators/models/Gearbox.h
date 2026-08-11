#ifndef CHRONO_ACTUATORS_MODEL_GEARBOX_H
#define CHRONO_ACTUATORS_MODEL_GEARBOX_H

namespace chrono {
namespace actuators {
namespace model {

/**
 * @brief Composable gearbox parameter block.
 *
 * This is intentionally lightweight so actuator models can compose gearbox
 * effects without depending on simulation-engine types.
 */
struct Gearbox {
    double ratio = 1.0;               ///< Output/input speed ratio.
    double efficiency = 1.0;          ///< Forward power-transfer efficiency [0,1].
    double viscous_friction = 0.0;    ///< Lumped viscous loss coefficient.
    double coulomb_friction = 0.0;    ///< Lumped Coulomb friction magnitude.
};

}  // namespace model
}  // namespace actuators
}  // namespace chrono

#endif
