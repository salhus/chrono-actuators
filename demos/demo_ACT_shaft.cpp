// =============================================================================
// demo_ACT_shaft
//
// LinearDamperModel bound to a ChShaft driveline via ChActuatorShaft.
// Demonstrates one model / many attachment types.
//
// Note: ChActuatorShaft requires a zero-state model.  Stateful stiff models
// (e.g. ElectricActuatorModel) must use ChActuatorDynamics for monolithic
// integration with the Chrono system.  See demo_ACT_electric for that path.
// =============================================================================

#include <cstdio>
#include <memory>

#include "chrono/core/ChTypes.h"
#include "chrono/physics/ChSystemNSC.h"
#include "chrono/physics/ChShaft.h"
#include "chrono/physics/ChShaftsMotorLoad.h"
#include "chrono_actuators/chrono/ChActuatorShaft.h"
#include "chrono_actuators/models/LinearDamperModel.h"

int main() {
    using namespace chrono;
    using namespace chrono::actuators;

    ChSystemNSC sys;

    auto shaft1 = chrono_types::make_shared<ChShaft>();
    shaft1->SetInertia(0.01);
    sys.AddShaft(shaft1);

    auto shaft2 = chrono_types::make_shared<ChShaft>();
    shaft2->SetInertia(0.1);
    sys.AddShaft(shaft2);

    auto motor_link = chrono_types::make_shared<ChShaftsMotorLoad>();
    motor_link->Initialize(shaft1, shaft2);
    sys.Add(motor_link);

    // Zero-state damper: zero-state model, safe for ChActuatorShaft.
    auto model = std::make_shared<LinearDamperModel>(0.05);

    ChActuatorShaft actuator(model, motor_link);

    ActuatorCommand cmd;
    cmd.effort  = 0.0;  // damper effort is purely velocity-derived
    cmd.enabled = true;

    std::printf("%-10s  %-14s  %-12s\n", "time[s]", "shaft2_omega", "effort[N·m]");

    const double dt = 1e-3;
    for (int i = 0; i < 500; ++i) {
        actuator.Advance(cmd, sys.GetChTime(), dt);
        sys.DoStepDynamics(dt);
        if (i % 50 == 0) {
            std::printf("%-10.3f  %-14.4f  %-12.4f\n",
                        sys.GetChTime(),
                        shaft2->GetPosDt(),
                        actuator.GetTelemetry().effort);
        }
    }
    return 0;
}
