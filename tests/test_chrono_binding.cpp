// =============================================================================
// Test: Chrono binding layer — TSDA functor re-query purity, sign convention.
// =============================================================================

#include <cassert>
#include <cmath>
#include <iostream>
#include <memory>
#include <sstream>

#include "chrono/core/ChTypes.h"
#include "chrono/physics/ChBody.h"
#include "chrono/physics/ChLinkTSDA.h"
#include "chrono/physics/ChSystemNSC.h"
#include "chrono/physics/ChUpdateFlags.h"
#include "chrono/solver/ChDirectSolverLS.h"
#include "chrono/timestepper/ChTimestepperImplicit.h"
#include "chrono_actuators/chrono/ChActuatorDynamics.h"
#include "chrono_actuators/chrono/ChActuatorFunctorTSDA.h"
#include "chrono_actuators/models/ElectricActuatorModel.h"
#include "chrono_actuators/models/LinearDamperModel.h"

using namespace chrono;
using namespace chrono::actuators;

// ---------------------------------------------------------------------------
// TSDA functor: bit-identical re-query (Invariant A at binding level)
// ---------------------------------------------------------------------------
static void test_tsda_requery_purity() {
    auto model = std::make_shared<LinearDamperModel>(300.0);
    ActuatorCommand cmd;
    cmd.enabled = true;

    auto functor = std::make_shared<ChActuatorFunctorTSDA>(model, cmd);

    auto sys = chrono_types::make_shared<ChSystemNSC>();
    auto body1 = chrono_types::make_shared<ChBody>();
    auto body2 = chrono_types::make_shared<ChBody>();
    sys->AddBody(body1);
    sys->AddBody(body2);

    auto tsda = chrono_types::make_shared<ChLinkTSDA>();
    tsda->Initialize(body1, body2, false, ChVector3d(0, 0, 0), ChVector3d(1, 0, 0));
    tsda->SetRestLength(1.0);
    tsda->RegisterForceFunctor(functor);
    sys->AddLink(tsda);

    const double f1 = functor->evaluate(1.0, 1.0, 1.5, 0.7, *tsda);
    const double f2 = functor->evaluate(1.0, 1.0, 1.5, 0.7, *tsda);
    assert(f1 == f2 && "TSDA functor: bit-identical re-query (Invariant A)");
}

// ---------------------------------------------------------------------------
// TSDA functor: sign convention — damper produces negative force for positive vel
// ---------------------------------------------------------------------------
static void test_tsda_sign_convention() {
    auto model = std::make_shared<LinearDamperModel>(100.0);
    ActuatorCommand cmd;
    auto functor = std::make_shared<ChActuatorFunctorTSDA>(model, cmd);

    auto sys = chrono_types::make_shared<ChSystemNSC>();
    auto b1  = chrono_types::make_shared<ChBody>();
    auto b2  = chrono_types::make_shared<ChBody>();
    sys->AddBody(b1);
    sys->AddBody(b2);
    auto tsda = chrono_types::make_shared<ChLinkTSDA>();
    tsda->Initialize(b1, b2, false, ChVector3d(0, 0, 0), ChVector3d(1, 0, 0));
    tsda->SetRestLength(1.0);
    tsda->RegisterForceFunctor(functor);
    sys->AddLink(tsda);

    const double f = functor->evaluate(0.0, 1.0, 1.0, 1.0, *tsda);
    assert(f < 0.0 && "TSDA: positive vel → negative force (damper opposes extension)");
}

class ConstantStatefulModel : public ActuatorModel {
  public:
    unsigned int GetNumStates() const override { return 1; }

    void SetInitialConditions(double* y0) const override {
        y0[0] = 1.0;
    }

    void CalculateRHS(double,
                      const double*,
                      const ActuatorState&,
                      const ActuatorCommand&,
                      double* rhs) const override {
        rhs[0] = 0.0;
    }

    double EffortFromStates(const double* y, const ActuatorState&) const override {
        return y[0];
    }

    double ComputeEffort(const ActuatorCommand&, const ActuatorState&) const override {
        return 0.0;
    }
};

static void test_dynamics_sign_convention() {
    auto sys = chrono_types::make_shared<ChSystemNSC>();

    auto ground = chrono_types::make_shared<ChBody>();
    ground->SetFixed(true);
    sys->AddBody(ground);

    auto body = chrono_types::make_shared<ChBody>();
    body->SetMass(1.0);
    body->SetPos(ChVector3d(1, 0, 0));
    sys->AddBody(body);

    auto dyn = chrono_types::make_shared<ChActuatorDynamics>(
        std::make_shared<ConstantStatefulModel>(),
        ground,
        body,
        true,
        ChVector3d(0, 0, 0),
        ChVector3d(0, 0, 0)
    );
    dyn->Initialize();
    dyn->FreezeCommand(ActuatorCommand{});
    sys->Add(dyn);

    const double x0 = body->GetPos().x();
    for (int i = 0; i < 20; ++i) {
        sys->DoStepDynamics(1e-3);
    }

    assert(body->GetPosDt().x() > 0.0 && "ChActuatorDynamics: positive effort must extend body 2");
    assert(body->GetPos().x() > x0 && "ChActuatorDynamics: positive effort must move body 2 outward");
}

static std::shared_ptr<ChActuatorDynamics> make_stiff_dynamics(std::shared_ptr<ChSystemNSC> sys) {
    auto ground = chrono_types::make_shared<ChBody>();
    ground->SetFixed(true);
    sys->AddBody(ground);

    auto body = chrono_types::make_shared<ChBody>();
    body->SetMass(1.0);
    body->SetPos(ChVector3d(1, 0, 0));
    sys->AddBody(body);

    ElectricActuatorParams p;
    p.R               = 1.0;
    p.L               = 1e-3;
    p.Kt              = 0.5;
    p.Ke              = 0.5;
    p.gear_ratio      = 1.0;
    p.gear_efficiency = 1.0;
    p.effort_max      = 100.0;

    auto dyn = chrono_types::make_shared<ChActuatorDynamics>(
        std::make_shared<ElectricActuatorModel>(p),
        ground,
        body,
        false,
        ChVector3d(0, 0, 0),
        ChVector3d(1, 0, 0)
    );
    dyn->Initialize();
    dyn->FreezeCommand(ActuatorCommand{});
    sys->Add(dyn);
    return dyn;
}

// ---------------------------------------------------------------------------
// ChActuatorDynamics stiff-solver warning: warn once for unsupported VI solvers
// ---------------------------------------------------------------------------
static void test_dynamics_stiff_warning_once_for_psor() {
    auto sys = chrono_types::make_shared<ChSystemNSC>();
    auto dyn = make_stiff_dynamics(sys);

    std::ostringstream captured;
    auto* old_buf = std::cerr.rdbuf(captured.rdbuf());

    dyn->Update(0.0, chrono::UpdateFlags::UPDATE_ALL);
    dyn->Update(0.0, chrono::UpdateFlags::UPDATE_ALL);

    std::cerr.rdbuf(old_buf);

    const std::string text = captured.str();
    assert(text.find("cannot consume KRM blocks") != std::string::npos);
    assert(text.find("demo_ACT_electric.cpp") != std::string::npos);
    assert(text.find("PSOR") != std::string::npos);
    assert(text.find("cannot consume KRM blocks") == text.rfind("cannot consume KRM blocks") &&
           "stiff-solver warning must be emitted at most once per instance");
}

// ---------------------------------------------------------------------------
// ChActuatorDynamics stiff-solver warning: no warning for direct solver setups
// ---------------------------------------------------------------------------
static void test_dynamics_stiff_no_warning_for_direct_solver() {
    auto sys = chrono_types::make_shared<ChSystemNSC>();

    auto solver = chrono_types::make_shared<ChSolverSparseLU>();
    sys->SetSolver(solver);
    solver->UseSparsityPatternLearner(true);
    solver->LockSparsityPattern(true);

    auto dyn = make_stiff_dynamics(sys);

    std::ostringstream captured;
    auto* old_buf = std::cerr.rdbuf(captured.rdbuf());

    dyn->Update(0.0, chrono::UpdateFlags::UPDATE_ALL);

    std::cerr.rdbuf(old_buf);

    assert(captured.str().empty() &&
           "stiff-solver warning must stay quiet when a direct solver is configured");
}

// ---------------------------------------------------------------------------
// ChActuatorDynamics sign convention with stiff model + direct solver
// Positive commanded effort must move the free body in the positive direction.
// This exercises the ODE path that was historically untested.
// ---------------------------------------------------------------------------
static void test_dynamics_stiff_sign_convention() {
    // A stiff model (IsStiff() == true) requires a direct solver.
    // The default PSOR solver cannot consume KRM blocks and will abort.
    auto sys = chrono_types::make_shared<ChSystemNSC>();

    auto solver = chrono_types::make_shared<ChSolverSparseLU>();
    sys->SetSolver(solver);
    solver->UseSparsityPatternLearner(true);
    solver->LockSparsityPattern(true);
    solver->SetVerbose(false);

    sys->SetTimestepperType(chrono::ChTimestepper::Type::EULER_IMPLICIT);
    auto integrator = std::static_pointer_cast<chrono::ChTimestepperEulerImplicit>(
        sys->GetTimestepper());
    integrator->SetMaxIters(50);
    integrator->SetAbsTolerances(1e-4, 1e2);

    auto ground = chrono_types::make_shared<ChBody>();
    ground->SetFixed(true);
    sys->AddBody(ground);

    auto body = chrono_types::make_shared<ChBody>();
    body->SetMass(1.0);
    body->SetPos(ChVector3d(1, 0, 0));
    sys->AddBody(body);

    ElectricActuatorParams p;
    p.R               = 1.0;
    p.L               = 1e-3;
    p.Kt              = 0.5;
    p.Ke              = 0.5;
    p.gear_ratio      = 1.0;
    p.gear_efficiency = 1.0;
    p.effort_max      = 100.0;

    auto model = std::make_shared<ElectricActuatorModel>(p);

    ActuatorCommand cmd;
    cmd.effort  = 10.0;  // positive effort → body should move in +x
    cmd.enabled = true;

    auto dyn = chrono_types::make_shared<ChActuatorDynamics>(
        model,
        ground,
        body,
        false,
        ChVector3d(0, 0, 0),
        ChVector3d(1, 0, 0)
    );
    dyn->Initialize();
    dyn->FreezeCommand(cmd);
    sys->Add(dyn);

    const double x0 = body->GetPos().x();
    for (int i = 0; i < 50; ++i) {
        sys->DoStepDynamics(1e-3);
    }

    // Sign convention: positive commanded effort must extend body 2
    // (move it further from body 1 in the direction of the actuator).
    assert(body->GetPosDt().x() > 0.0 &&
           "ChActuatorDynamics stiff: positive effort must give positive velocity");
    assert(body->GetPos().x() > x0 &&
           "ChActuatorDynamics stiff: positive effort must move body 2 outward");
    assert(dyn->GetEffort() > 0.0 &&
           "ChActuatorDynamics stiff: GetEffort() must be positive for positive command");
}

int main() {
    test_tsda_requery_purity();
    test_tsda_sign_convention();
    test_dynamics_sign_convention();
    test_dynamics_stiff_warning_once_for_psor();
    test_dynamics_stiff_no_warning_for_direct_solver();
    test_dynamics_stiff_sign_convention();
    return 0;
}
