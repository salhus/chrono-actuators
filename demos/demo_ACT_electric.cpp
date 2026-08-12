// =============================================================================
// demo_ACT_electric
//
// ElectricActuatorModel via ChActuatorDynamics driving a body pair.
// Demonstrates stiff-integration stability (IsStiff() == true) and the
// DC bus voltage limit that gives the model a real torque-speed characteristic.
//
// Two observable regimes:
//   1. Unsaturated: while Ke*omega_motor < V_bus, the back-EMF feedforward
//      keeps current pinned at the commanded value.  Effort is constant and
//      the body accelerates at a = F/m.
//   2. Saturated: once the motor back-EMF consumes the available bus voltage,
//      current falls as i = (V_bus - Ke*omega)/R.  Effort droops and the
//      body asymptotically approaches the motor no-load speed.
//
// V_bus is set to 6.0 V so the saturation knee falls well within the
// simulated range (~1.5 m/s output shaft), making both regimes visible in
// the printed table.
//
// REQUIREMENT: ElectricActuatorModel::IsStiff() returns true, so
// ChActuatorDynamics inserts KRM blocks into the system descriptor.
// ChSystemNSC's default PSOR solver is an iterative VI solver that cannot
// consume KRM blocks and will abort.  A direct solver (ChSolverSparseLU or
// ChSolverSparseQR) plus an implicit timestepper (EULER_IMPLICIT) is required
// for any stiff model bound through ChActuatorDynamics.
// =============================================================================

#include <cstdio>
#include <memory>

#include "chrono/core/ChTypes.h"
#include "chrono/physics/ChBody.h"
#include "chrono/physics/ChSystemNSC.h"
#include "chrono/solver/ChDirectSolverLS.h"
#include "chrono/timestepper/ChTimestepperImplicit.h"
#include "chrono_actuators/chrono/ChActuatorDynamics.h"
#include "chrono_actuators/models/ElectricActuatorModel.h"

int main() {
    using namespace chrono;
    using namespace chrono::actuators;

    ChSystemNSC sys;

    // -------------------------------------------------------------------------
    // Stiff ODE solver setup
    // ElectricActuatorModel::IsStiff() == true → ChActuatorDynamics injects a
    // KRM block into the system descriptor.  The default PSOR iterative VI
    // solver cannot consume KRM blocks; a direct sparse solver is required.
    // -------------------------------------------------------------------------
    auto solver = chrono_types::make_shared<ChSolverSparseLU>();
    sys.SetSolver(solver);
    solver->UseSparsityPatternLearner(true);
    solver->LockSparsityPattern(true);
    solver->SetVerbose(false);

    sys.SetTimestepperType(ChTimestepper::Type::EULER_IMPLICIT);
    auto integrator = std::static_pointer_cast<ChTimestepperEulerImplicit>(sys.GetTimestepper());
    integrator->SetMaxIters(50);
    integrator->SetAbsTolerances(1e-4, 1e2);

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
    p.V_bus           = 6.0;   // low bus so knee at ~1.5 m/s output speed
    p.effort_max      = 10.0;

    auto model = std::make_shared<ElectricActuatorModel>(p);

    // Print torque-speed endpoints so the reader can check the table asymptotes.
    std::printf("Stall effort  = %.4f N  (at omega_out = 0)\n", model->GetStallEffort());
    std::printf("No-load speed = %.4f m/s (output shaft asymptote)\n\n", model->GetNoLoadSpeed());

    // enabled must be set explicitly; default is false (safe for HIL, but
    // a silent no-op in pure simulation if forgotten).
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
    dyn->Initialize();
    dyn->FreezeCommand(cmd);
    sys.Add(dyn);

    // Print columns: time, position, velocity, winding current (state[0]), effort.
    // Positive effort → arm accelerates in +x direction.
    // Unsaturated: current pinned, effort constant, velocity ramps linearly.
    // Saturated: current falls as i = (V_bus - Ke*omega_motor)/R; effort droops;
    // velocity approaches no-load speed asymptotically.
    std::printf("%-10s  %-14s  %-14s  %-12s  %-12s\n",
                "time[s]", "arm_pos_x[m]", "arm_vel_x[m/s]", "current[A]", "effort[N]");

    for (int i = 0; i < 1000; ++i) {
        sys.DoStepDynamics(1e-3);
        if (i % 50 == 0) {
            const auto& states = dyn->GetStates();
            const double current = (states.size() > 0) ? states[0] : 0.0;
            std::printf("%-10.3f  %-14.4f  %-14.4f  %-12.4f  %-12.4f\n",
                        sys.GetChTime(),
                        arm->GetPos().x(),
                        arm->GetPosDt().x(),
                        current,
                        dyn->GetEffort());
        }
    }
    return 0;
}
