// =============================================================================
// demo_ACT_shaft
//
// Same ElectricActuatorModel bound to a ChShaft driveline via ChActuatorShaft.
// Demonstrates one model / many attachment types.
// =============================================================================

#include <cstdio>
#include <memory>

#include "chrono/core/ChTypes.h"
#include "chrono/physics/ChSystemNSC.h"
#include "chrono/physics/ChShaft.h"
#include "chrono/physics/ChShaftsMotorTorque.h"
#include "chrono_actuators/chrono/ChActuatorShaft.h"
#include "chrono_actuators/models/ElectricActuatorModel.h"

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

    auto motor_link = chrono_types::make_shared<ChShaftsMotorTorque>();
    motor_link->Initialize(shaft1, shaft2);
    sys.AddLink(motor_link);

    ElectricActuatorParams p;
    p.R               = 0.5;
    p.L               = 5e-4;
    p.Kt              = 0.15;
    p.Ke              = 0.15;
    p.gear_ratio      = 10.0;
    p.gear_efficiency = 0.92;

    auto model = std::make_shared<ElectricActuatorModel>(p);

    ChActuatorShaft actuator(model, motor_link);

    ActuatorCommand cmd;
    cmd.effort  = 0.5;
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
