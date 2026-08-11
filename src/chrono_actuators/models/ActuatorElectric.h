#ifndef CHRONO_ACTUATORS_MODEL_ACTUATOR_ELECTRIC_H
#define CHRONO_ACTUATORS_MODEL_ACTUATOR_ELECTRIC_H

#include "chrono_actuators/models/ActuatorModel.h"
#include "chrono_actuators/models/Gearbox.h"

namespace chrono {
namespace actuators {
namespace model {

/**
 * @brief Placeholder geared-DC electric actuator model interface.
 *
 * Intended parameterization (v0.2):
 * - torque constant and back-EMF constant
 * - winding resistance/inductance (with quasi-static electrical default)
 * - gearbox ratio/efficiency and friction losses
 * - output viscous + Coulomb friction
 * - effort/torque saturation limits
 */
class ActuatorElectric : public ActuatorModel {
  public:
    struct Parameters {
        double torque_constant = 0.0;
        double back_emf_constant = 0.0;
        double winding_resistance = 0.0;
        double winding_inductance = 0.0;
        Gearbox gearbox{};
        double output_viscous_friction = 0.0;
        double output_coulomb_friction = 0.0;
        double effort_limit = 0.0;
    };

    ~ActuatorElectric() override = default;

    /**
     * @brief Return immutable parameter view.
     */
    virtual const Parameters& GetParameters() const = 0;

    Effort ComputeEffort(const ActuatorCommand& command, const ActuatorState& state, double dt) override = 0;
};

}  // namespace model
}  // namespace actuators
}  // namespace chrono

#endif
