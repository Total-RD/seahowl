#include "seahowl/aero/turbine_aero.h"

using namespace seahowl::aero;
using seahowl::env::FluidModel;

TurbineAero::TurbineAero() {
    rna = RotorNacelleAssemblyAero();
    tower = TowerAero();
}

void TurbineAero::build() {
    rna.build();
    tower.build();
    if (foundation) {
        foundation->build();
    }
}

void TurbineAero::initialize(double time, double dt) {}

void TurbineAero::compute_fluid_loads(const FluidModel& wind_model, double time) {
    rna.compute_fluid_loads(wind_model, time);
    tower.compute_fluid_loads(wind_model, time);
    if (foundation) {
        foundation->compute_fluid_loads(wind_model, time);
    }
}
