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
}

void TurbineAero::initialize(double time, double dt) {}

void TurbineAero::compute_aero_loads(const FluidModel& wind_model, double time) {
    rna.rotor->compute_aero_loads(wind_model, time);
    tower.compute_aero_loads(wind_model, time);
}
