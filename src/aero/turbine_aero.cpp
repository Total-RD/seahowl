#include "seahowl/aero/turbine_aero.h"

using namespace seahowl::aero;

TurbineAero::TurbineAero() {
    rotor = RotorAero();
    tower = TowerAero();
}

void TurbineAero::build() {
    rotor.build();
    tower.build();
}

// void TurbineAero::compute_aero_loads(seahowl::aero::WindModel& wind_model, double time) {
//#ifdef HAVE_AERODYN
//     if (use_aerodyn) {
//         aerodyn->calcul(time, *this);
//         rotor.aero.compute_wind_loads_aerodyn(aerodyn->pImpl.MeshFrc, wind_model, time, tower.aero, true, true,
//         true);
//     } else {
//         rotor.aero.compute_wind_loads_bemt(wind_model, time, tower.aero, true, true, true);
//     }
//#else
//     // rotor.compute_aero_loads(wind_model, time, tower, true, true, true);
//#endif
//     tower.compute_aero_loads(wind_model, time);
// }
