#include "seahowl/core/floater.h"

#include "seahowl/core/mooring.h"
#include "seahowl/elasto/floater_elasto.h"
#include "seahowl/elasto/mooring_elasto.h"
#include "seahowl/hydro/floater_hydro.h"
#include "seahowl/hydro/mooring_hydro.h"

#include <memory>
#include <vector>
#include <spdlog/spdlog.h>

using namespace seahowl::core;
using namespace seahowl::elasto;
using namespace seahowl::hydro;

Floater::Floater(std::shared_ptr<seahowl::elasto::FloaterElasto> elasto,
                 std::shared_ptr<seahowl::hydro::FloaterHydro> hydro)
    : ComponentDynamic(elasto, hydro), elasto(*elasto), hydro(*hydro) {
    mooring_system = std::make_unique<MooringSystem>(elasto->mooring_system, hydro->mooring_system);
}

void Floater::initialize_this(double time, double dt) {
    mooring_system->initialize(time, dt);

    elasto.initialize();

    spdlog::info("Initialized floater of total mass {:.4}kg (with moorings).", elasto.get_mass());
}

void Floater::prestep(double time, double dt) {
    mooring_system->prestep(time, dt);
}

void Floater::poststep(double time, double dt) {
    mooring_system->poststep(time, dt);
}

void Floater::apply_env_model(seahowl::env::EnvModel& env_model, double time) {
    mooring_system->apply_env_model(env_model, time);
}

void Floater::apply_soil_model(seahowl::env::EnvModel& env_model, double time) {
    mooring_system->apply_soil_model(env_model, time);
}

void Floater::build() {
    // build elasto
    elasto.build();
}
