#include "seahowl/fluid/aero/system_aero.h"

using namespace seahowl::aero;

void SystemAero::build() {
    // build all turbines
    for (auto& turbine : turbines) {
        turbine->build();
    }
    // build all extra components
    for (auto& component : components) {
        component->build();
    }
}

void SystemAero::compute_env_loads(const env::EnvModel& env_model, double time) {
    for (auto& turbine : turbines) {
        // compute forces from fluid model
        turbine->compute_env_loads(env_model, time);
    }
    for (auto& component : components) {
        // compute forces from fluid model
        component->compute_env_loads(env_model, time);
    }
}
