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
        turbine.compute_wind_loads(*wind_model, time);

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

void System::assemble(std::shared_ptr<seahowl::elasto::SystemElasto> system,
                      std::shared_ptr<seahowl::elasto::MeshElasto> mesh) {
    system_elasto = system;
    for (auto& turbine : turbines) {
        turbine.assemble(system, mesh);
    }
}

double System::get_time() {
    return system_elasto->get_time();
}
