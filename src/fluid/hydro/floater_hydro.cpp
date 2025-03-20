#include "seahowl/fluid/hydro/floater_hydro.h"
#include "seahowl/fluid/hydro/mooring_hydro.h"
#include "seahowl/env/env_model.h"


using namespace seahowl;
using namespace seahowl::hydro;
using namespace seahowl::env;

FloaterHydro::FloaterHydro() {
    mooring_system = std::make_unique<MooringSystemHydro>();
}

void FloaterHydro::build() {
    mooring_system->build();
}

void FloaterHydro::compute_env_loads(const EnvModel& env_model, double time) {
    mooring_system->compute_env_loads(env_model, time);
}
