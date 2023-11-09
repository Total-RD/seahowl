#include "seahowl/core/turbine_floating.h"
#include "seahowl/elasto/chrono_adapters.h"
#include "seahowl/elasto/mooring_elasto.h"
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

    // prestep for moorings
    for (auto& mooring : elasto.mooring_system->moorings) {
        mooring->prestep(time, dt);
    }

    if (elasto.floater) {
        elasto.floater->prestep(time, dt);
    }
}

void TurbineFloating::poststep(double time, double dt) {
    // parent class poststep
    Turbine::poststep(time, dt);
}

void TurbineFloating::build() {
    elasto.build();
    aero.build();
}

void TurbineFloating::apply_fluid_model(seahowl::env::FluidModel& fluid_model, double time) {
    Turbine::apply_fluid_model(fluid_model, time);
    for (auto& mooring : elasto.mooring_system->moorings) {
        try {
            dynamic_cast<seahowl::elasto::MooringElastoFEA&>(*mooring).compute_hydro_loads(Vector3d(0., 0., -9.81),
                                                                                           1025.);
        } catch (const std::bad_cast& e) {
            // do nothing
        }
    }
}

void TurbineFloating::apply_soil_model(seahowl::env::SoilModel& soil_model, double time) {
    for (auto& mooring : elasto.mooring_system->moorings) {
        try {
            dynamic_cast<seahowl::elasto::MooringElastoFEA&>(*mooring).compute_seabed_loads(soil_model);
        } catch (const std::bad_cast& e) {
            // do nothing
        }
    }
}
