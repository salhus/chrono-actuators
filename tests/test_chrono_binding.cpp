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

int main() {
    test_tsda_requery_purity();
    test_tsda_sign_convention();
    return 0;
}
