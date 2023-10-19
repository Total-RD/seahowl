#include "seahowl/core/turbine_floating.h"
#include "seahowl/elasto/chrono_adapters.h"
#include "seahowl/aero/turbine_aero.h"

#include <spdlog/spdlog.h>

using namespace seahowl::core;
using namespace seahowl::servo;
using namespace seahowl::elasto;

TurbineFloating::TurbineFloating(seahowl::elasto::TurbineFloatingElasto& elasto, seahowl::aero::TurbineAero& aero)
    : elasto(elasto), Turbine(elasto, aero) {}

void TurbineFloating::initialize(double time, double dt) {
    // parent class initialize
    Turbine::initialize(time, dt);

    if (elasto.floater) {
        elasto.floater->initialize();
    }

    spdlog::info("Initialized turbine of total mass {:.4}kg.", elasto.get_mass());
}

void TurbineFloating::prestep(double time, double dt) {
    // parent class prestep
    Turbine::prestep(time, dt);
}

void TurbineFloating::poststep(double time, double dt) {
    // parent class poststep
    Turbine::poststep(time, dt);
}

void TurbineFloating::build() {
    elasto.build();
    aero.build();
}
