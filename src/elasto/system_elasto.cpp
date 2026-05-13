// SPDX-License-Identifier: Apache-2.0
// Copyright 2022–2026 TotalEnergies
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//

#include "seahowl/elasto/system_elasto.h"

// Third-party libraries
#include <spdlog/spdlog.h>

using namespace seahowl::elasto;

void SystemElasto::add(std::shared_ptr<ComponentElasto> component) {
    if (std::find(components.begin(), components.end(), component) == components.end()) {
        components.push_back(component);
    } else
        spdlog::warn("Components Elasto already exists in the system, not adding again.");
};

void SystemElasto::add(std::shared_ptr<TurbineElasto> turbine) {
    if (std::find(turbines.begin(), turbines.end(), turbine) == turbines.end()) {
        turbines.push_back(turbine);
    } else
        spdlog::warn("Turbine Elasto already exists in the system, not adding again.");
}

void SystemElasto::build() {
    // build all turbines
    for (auto& turbine : turbines) {
        turbine->build();
    }
    // build all extra components
    for (auto& component : components) {
        component->build();
    }
}

void SystemElasto::translate(const Vector3d& translation_vector) const {
    for (const auto& turbine : turbines) {
        turbine->translate(translation_vector);
    }
    for (const auto& component : components) {
        component->translate(translation_vector);
    }
}

void SystemElasto::rotate(double angle, const Vector3d& axis) const {
    for (const auto& turbine : turbines) {
        turbine->rotate(angle, axis);
    }
    for (const auto& component : components) {
        component->rotate(angle, axis);
    }
}

double SystemElasto::get_mass() const {
    double mass = 0.0;
    for (auto& turbine : turbines) {
        mass += turbine->get_mass();
    }
    for (auto& component : components) {
        mass += component->get_mass();
    }
    return mass;
}

void SystemElasto::assemble_this(seahowl::elasto::SystemElasto& system) {
    if (&system != &*this) {
        throw std::runtime_error("Cannot assemble a SystemElasto instance using another SystemElasto instance.");
    }
    assemble();
}
