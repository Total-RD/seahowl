#include "seahowl/aero/turbine_aero.h"

using namespace seahowl::aero;

TurbineAero::TurbineAero() {
    rna = RotorNacelleAssemblyAero();
    tower = TowerAero();
}

void TurbineAero::build() {
    rna.build();
    tower.build();
}

void TurbineAero::initialize(double time, double dt) {
#ifdef HAVE_AERODYN
    if (use_aerodyn) {
        aerodyn->initialize(time, dt, *this);
    }
#endif
}

void TurbineAero::compute_aero_loads(seahowl::aero::WindModel& wind_model, double time) {
#ifdef HAVE_AERODYN
    if (use_aerodyn) {
        aerodyn->calcul(time, *this);
        rna.compute_aero_loads(aerodyn->pImpl.MeshFrc, wind_model, time, tower, true, true, true);
    } else {
        rna.compute_aero_loads(wind_model, time, tower, true, true, true);
    }
#else
    rna.compute_aero_loads(wind_model, time, tower, true, true, true);
#endif
    tower.compute_aero_loads(wind_model, time);
}
