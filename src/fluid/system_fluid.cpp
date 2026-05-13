// SPDX-License-Identifier: Apache-2.0
// Copyright 2022–2026 TotalEnergies
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//

#include "seahowl/fluid/system_fluid.h"

// SEAHOWL headers
#include "seahowl/env/env_model.h"

// Third-party libraries
#include <spdlog/spdlog.h>

using namespace seahowl::fluid;

void SystemFluid::add(std::shared_ptr<TurbineFluid> turbine) {
    if (std::find(turbines.begin(), turbines.end(), turbine) == turbines.end()) {
        turbines.push_back(turbine);
    } else
        spdlog::warn("Turbine fluid already exists in the system, not adding again.");
}

void SystemFluid::add(std::shared_ptr<seahowl::fluid::ComponentFluid> component) {
    if (std::find(components.begin(), components.end(), component) == components.end()) {
        components.push_back(component);
    } else
        spdlog::warn("Component fluid already exists in the system, not adding again.");
}

void SystemFluid::build() {
    // build all turbines
    for (auto& turbine : turbines) {
        turbine->build();
    }
    // build all extra components
    for (auto& component : components) {
        component->build();
    }
}

void SystemFluid::compute_env_loads(const env::EnvModel& env_model, double time) {
    for (auto& turbine : turbines) {
        // compute forces from fluid model
        turbine->compute_env_loads(env_model, time);
    }
    for (auto& component : components) {
        // compute forces from fluid model
        component->compute_env_loads(env_model, time);
    }
}
