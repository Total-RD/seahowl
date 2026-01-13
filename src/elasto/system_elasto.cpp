#include "seahowl/elasto/system_elasto.h"

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
