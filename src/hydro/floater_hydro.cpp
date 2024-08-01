#include "seahowl/hydro/floater_hydro.h"

#include "seahowl/hydro/mooring_hydro.h"
#include "seahowl/env/fluid_models.h"

using namespace seahowl;
using namespace seahowl::hydro;
using namespace seahowl::env;

FloaterHydro::FloaterHydro() {
    mooring_system = std::make_unique<MooringSystemHydro>();
}

void FloaterHydro::build() {
    mooring_system->build();
}

void FloaterHydro::compute_fluid_loads(const FluidModel& fluid_model, double time) {
    mooring_system->compute_fluid_loads(fluid_model, time);
}
