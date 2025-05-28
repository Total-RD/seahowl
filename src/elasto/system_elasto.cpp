#include "seahowl/elasto/system_elasto.h"

using namespace seahowl::elasto;

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
    for (auto& turbine : turbines) {
        turbine->translate(translation_vector);
    }
    for (auto& component : components) {
        component->translate(translation_vector);
    }
}

void SystemElasto::rotate(double angle, const Vector3d& axis) const {
    for (auto& turbine : turbines) {
        turbine->rotate(angle, axis);
    }
    for (auto& component : components) {
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
