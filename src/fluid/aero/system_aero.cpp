#include "seahowl/fluid/aero/system_aero.h"

#include <spdlog/spdlog.h>

using namespace seahowl::aero;

void SystemAero::add(std::shared_ptr<TurbineAero> turbine) {
    if (std::find(turbines.begin(), turbines.end(), turbine) == turbines.end()) {
        turbines.push_back(turbine);
    } else
        spdlog::warn("Turbine aero already exists in the system, not adding again.");
}

void SystemAero::add(std::shared_ptr<seahowl::ComponentFluid> component) {
    if (std::find(components.begin(), components.end(), component) == components.end()) {
        components.push_back(component);
    } else
        spdlog::warn("Component aero already exists in the system, not adding again.");
}

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
