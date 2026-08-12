// =============================================================================
// Test: Chrono binding layer — TSDA functor re-query purity, sign convention.
// =============================================================================

#include <cassert>
#include <cmath>
#include <memory>

#include "chrono/core/ChTypes.h"
#include "chrono/physics/ChBody.h"
#include "chrono/physics/ChLinkTSDA.h"
#include "chrono/physics/ChSystemNSC.h"
#include "chrono_actuators/chrono/ChActuatorDynamics.h"
#include "chrono_actuators/chrono/ChActuatorFunctorTSDA.h"
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

int main() {
    test_tsda_requery_purity();
    test_tsda_sign_convention();
    test_dynamics_sign_convention();
    return 0;
}
