#include "seahowl/elasto/turbine_floating_elasto.h"

#include "seahowl/elasto/system_elasto.h"
#include "seahowl/elasto/chrono_adapters.h"

#include <iostream>

using namespace seahowl::elasto;

TurbineFloatingElasto::TurbineFloatingElasto() : TurbineElasto() {
    link_floater_tower = std::make_unique<seahowl::elasto::LinkChrono>();
    mooring_system = std::make_unique<MooringSystem>();
}

void TurbineFloatingElasto::assemble(SystemElasto& system) {
    // assemble floater first
    if (floater) {
        // assemble floater
        floater->assemble(system);
        // make floater-tower connection
        auto& body_floater = floater->get_tower_connection_body();
        auto& node_tower = *(tower.nodes.front().get());
        link_floater_tower->initialize(body_floater, node_tower);
        link_floater_tower->set_constraints(true, true, true, true, true, true);
        // add link to system
        system.add(*(link_floater_tower.get()));
    }

    // moorings
    mooring_system->assemble(system);

    // assemble parent class
    TurbineElasto::assemble(system);
}

void TurbineFloatingElasto::build() {
    if (floater) {
        floater->build();
    }
    mooring_system->build();

    // parent class build
    TurbineElasto::build();
}

void TurbineFloatingElasto::translate(const Vector3d& translation_vector) const {
    // parent class translate
    TurbineElasto::translate(translation_vector);
    // floater translate
    if (floater) {
        floater->translate(translation_vector);
    }
}

void TurbineFloatingElasto::rotate(double angle, const Vector3d& axis) const {
    // parent class rotate
    TurbineElasto::rotate(angle, axis);
    // floater rotate
    if (floater) {
        floater->rotate(angle, axis);
    }
}

double TurbineFloatingElasto::get_mass() const {
    double total_mass = 0.0;
    // turbine
    total_mass += TurbineElasto::get_mass();
    // floater
    if (floater) {
        total_mass += floater->get_mass();
    }
    return total_mass;
}
