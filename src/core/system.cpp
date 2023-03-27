#include <seahowl/core/system.h>

using namespace seahowl::core;

System::System() {}

void System::init(double time, double dt) {
    for (auto& turbine : turbines) {
        turbine.init(time, dt);
    }
}

void System::prestep(double time, double dt) {
    for (auto& turbine : turbines) {
        // compute forces on rotor and tower
        turbine.aero.compute_aero_loads(*wind_model, time);
        // turbine prestep (accumulates loads from aero to elasto)
        turbine.prestep(time, dt);
    }
}

void System::step(double dt) {
    system_elasto->step(dt);
}

void System::poststep(double time, double dt) {
    for (auto& turbine : turbines) {
        // turbine poststep
        turbine.poststep(time, dt);
    }
}

void System::assemble() {
    for (auto& turbine : turbines) {
        turbine.elasto.assemble(*(system_elasto.get()));
    }
}

double System::get_time() {
    return system_elasto->get_time();
}
