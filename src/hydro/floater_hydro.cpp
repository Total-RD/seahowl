#include "seahowl/hydro/floater_hydro.h"

#include "seahowl/hydro/mooring_hydro.h"

using namespace seahowl;
using namespace seahowl::hydro;

FloaterHydro::FloaterHydro() {
    mooring_system = std::make_unique<MooringSystemHydro>();
}
