// =============================================================================
// demo_ACT_electric
//
// ElectricActuatorModel via ChActuatorDynamics driving a body pair.
// Demonstrates stiff-integration stability (IsStiff() == true).
// =============================================================================

#include <cstdio>
#include <memory>

#include "chrono/core/ChTypes.h"
#include "chrono/physics/ChBody.h"
#include "chrono/physics/ChSystemNSC.h"
#include "chrono_actuators/chrono/ChActuatorDynamics.h"
#include "chrono_actuators/models/ElectricActuatorModel.h"

int main() {
    using namespace chrono;
    using namespace chrono::actuators;

    ChSystemNSC sys;

    auto ground = chrono_types::make_shared<ChBody>();
    ground->SetFixed(true);
    sys.AddBody(ground);

    auto arm = chrono_types::make_shared<ChBody>();
    arm->SetMass(2.0);
    arm->SetPos(ChVector3d(0.5, 0, 0));
    sys.AddBody(arm);

    ElectricActuatorParams p;
    p.R               = 1.0;
    p.L               = 1e-3;
    p.Kt              = 0.2;
    p.Ke              = 0.2;
    p.gear_ratio      = 20.0;
    p.gear_efficiency = 0.9;
    p.effort_max      = 10.0;

    auto model = std::make_shared<ElectricActuatorModel>(p);

    ActuatorCommand cmd;
    cmd.effort  = 5.0;
    cmd.enabled = true;

    auto dyn = chrono_types::make_shared<ChActuatorDynamics>(
        model,
        ground,
        arm,
        false,
        ChVector3d(0, 0, 0),
        ChVector3d(0.5, 0, 0)
    );
    dyn->FreezeCommand(cmd);
    sys.Add(dyn);

    std::printf("%-10s  %-14s  %-12s\n", "time[s]", "arm_pos_x[m]", "effort[N]");

    for (int i = 0; i < 500; ++i) {
        sys.DoStepDynamics(1e-3);
        if (i % 50 == 0) {
            std::printf("%-10.3f  %-14.4f  %-12.4f\n",
                        sys.GetChTime(),
                        arm->GetPos().x(),
                        dyn->GetEffort());
        }
    }
    return 0;
}
