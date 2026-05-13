// SPDX-License-Identifier: Apache-2.0
// Copyright 2022–2026 TotalEnergies
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//

#include "seahowl/core/tower.h"

// SEAHOWL headers
#include "seahowl/commons/utils.h"
#include "seahowl/elasto/tower_elasto.h"
#include "seahowl/fluid/aero/tower_aero.h"

// Third-party libraries
#include <spdlog/spdlog.h>

using namespace seahowl::core;
using namespace seahowl::elasto;
using namespace seahowl::fluid::aero;

Tower::Tower(std::shared_ptr<seahowl::elasto::TowerElasto> elasto,
             std::shared_ptr<seahowl::fluid::aero::TowerAero> aero)
    : ComponentDynamic(elasto, aero), ComponentElastoFluid(elasto, aero), elasto(*elasto), aero(*aero) {}

void Tower::initialize_this(double time, double dt) {
    aero.initialize(time, dt);

    // mappings
    compute_mapping_fluid2elasto();
    compute_mapping_elasto2fluid();
    // update position of aero points
    update_positions_aero();

    spdlog::info("Initialized tower of total mass {:.4}kg with {} elasto and {} aero elements.", elasto.get_mass(),
                 elasto.elements.size(), aero.elements.size());
}

void Tower::prestep(double time, double dt) {
    // update loads on elasto part
    update_loads_elasto();
}

void Tower::poststep(double time, double dt) {
    // update position of aero points
    update_positions_aero();
}

void Tower::apply_env_model(seahowl::env::EnvModel& env_model, double time) {
    aero.compute_env_loads(env_model, time);
}

void Tower::build() {
    // build
    elasto.build();
    aero.build();
}

void Tower::set_discretization_elasto(const std::vector<double>& fractions) {
    elasto.discretization_fractions = fractions;
}

void Tower::set_discretization_aero(const std::vector<double>& fractions) {
    aero.discretization_fractions = fractions;
}

void Tower::update_positions_aero() {
    for (int ii = 0; ii < aero.nodes.size(); ii++) {
        auto& node_aero = aero.nodes[ii];

        // update position and rotation of aero elements
        int elasto_element_index = mapping_fluid2elasto_nodes[ii].index;
        double eta = mapping_fluid2elasto_nodes[ii].eta;
        auto entity = elasto.get_entity_along_component_slerp(eta, elasto_element_index);
        node_aero.set_rotation(entity.get_rotation());
        node_aero.set_position(entity.get_position());
        node_aero.set_velocity(entity.get_velocity());
        node_aero.set_rotational_velocity(entity.get_rotational_velocity());
        node_aero.set_acceleration(entity.get_acceleration());
        node_aero.set_rotational_acceleration(entity.get_rotational_acceleration());
    }
}

void Tower::update_loads_elasto() {
    elasto.reset_loads();
    if (aero.has_nodal_distributed_loads) {
        // loads at nodes are distributed loads
        // get integrated loads at center of element
        if (aero.elements.size() != mapping_fluid2elasto_elements.size()) {
            throw std::runtime_error("Tower: length of vector of elements (" + std::to_string(aero.elements.size()) +
                                     " and length of aero to elasto mapping(" +
                                     std::to_string(mapping_fluid2elasto_elements.size()) + ") do not match.");
        }
        auto offset = Vector3d(0.0, 0.0, 0.0);
        for (int ii = 0; ii < aero.elements.size(); ii++) {
            elasto.accumulate_element_load(aero.elements[ii].get_load_noacc(), Vector3d(0.0, 0.0, 0.0),
                                           mapping_fluid2elasto_elements[ii].index,
                                           mapping_fluid2elasto_elements[ii].eta, offset);
            elasto.accumulate_mass_matrix(aero.elements[ii].get_added_mass_matrix(),
                                          mapping_fluid2elasto_elements[ii].index,
                                          mapping_fluid2elasto_elements[ii].eta);
        }
    } else {
        // loads at nodes are point loads
        if (aero.nodes.size() != mapping_fluid2elasto_nodes.size()) {
            throw std::runtime_error("Tower: length of vector of nodes (" + std::to_string(aero.nodes.size()) +
                                     " and length of aero to elasto mapping(" +
                                     std::to_string(mapping_fluid2elasto_nodes.size()) + ") do not match.");
        }
        auto offset = Vector3d(0.0, 0.0, 0.0);
        for (int ii = 0; ii < aero.nodes.size(); ii++) {
            elasto.accumulate_element_load(aero.nodes[ii].load_noacc, Vector3d(0.0, 0.0, 0.0),
                                           mapping_fluid2elasto_nodes[ii].index, mapping_fluid2elasto_nodes[ii].eta,
                                           offset);
            elasto.accumulate_mass_matrix(aero.nodes[ii].added_mass_matrix, mapping_fluid2elasto_nodes[ii].index,
                                          mapping_fluid2elasto_nodes[ii].eta);
        }
    }
}
