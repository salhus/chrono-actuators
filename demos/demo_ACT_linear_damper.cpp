// =============================================================================
// demo_ACT_linear_damper
//
// PTO damper on a TSDA between two bodies. Reports absorbed power.
// =============================================================================

#include <cmath>
#include <cstdio>
#include <memory>

#include "chrono/core/ChTypes.h"
#include "chrono/physics/ChBody.h"
#include "chrono/physics/ChLinkTSDA.h"
#include "chrono/physics/ChSystemNSC.h"
#include "chrono_actuators/chrono/ChActuatorFunctorTSDA.h"
#include "chrono_actuators/models/LinearDamperModel.h"

int main() {
    using namespace chrono;
    using namespace chrono::actuators;

    ChSystemNSC sys;
    sys.SetGravitationalAcceleration(ChVector3d(0, -9.81, 0));

    auto ground = chrono_types::make_shared<ChBody>();
    ground->SetFixed(true);
    sys.AddBody(ground);

    auto mass = chrono_types::make_shared<ChBody>();
    mass->SetMass(100.0);
    mass->SetPos(ChVector3d(0, -1.0, 0));
    sys.AddBody(mass);

    auto model   = std::make_shared<LinearDamperModel>(2000.0);
    ActuatorCommand cmd;
    cmd.enabled = true;
    auto functor = std::make_shared<ChActuatorFunctorTSDA>(model, cmd);

    auto tsda = chrono_types::make_shared<ChLinkTSDA>();
    tsda->Initialize(ground, mass, false, ChVector3d(0, 0, 0), ChVector3d(0, -1.0, 0));
    tsda->SetRestLength(1.0);
    tsda->RegisterForceFunctor(functor);
    sys.AddLink(tsda);

    const double dt = 1e-3;
    double total_energy = 0.0;

    std::printf("%-10s  %-12s  %-12s  %-12s\n", "time[s]", "pos[m]", "vel[m/s]", "power[W]");

    for (int i = 0; i < 2000; ++i) {
        sys.DoStepDynamics(dt);

        const double vel    = mass->GetPosDt().y();
        const double power  = functor->GetTelemetry().mechanical_power;
        total_energy += std::abs(power) * dt;

        if (i % 200 == 0) {
            std::printf("%-10.3f  %-12.4f  %-12.4f  %-12.2f\n",
                        sys.GetChTime(),
                        mass->GetPos().y(),
                        vel,
                        power);
        }
    }

    std::printf("\nTotal absorbed energy: %.4f J\n", total_energy);
    return 0;
}
