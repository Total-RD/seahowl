// SPDX-License-Identifier: Apache-2.0
// Copyright 2022–2026 TotalEnergies
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//

#include "seahowl/core/blade.h"

// SEAHOWL headers
#include "seahowl/commons/utils.h"
#include "seahowl/elasto/blade_elasto.h"
#include "seahowl/fluid/aero/blade_aero.h"

// Third-party libraries
#include <spdlog/spdlog.h>

// Standard library
#include <memory>

using namespace seahowl::core;
using namespace seahowl::elasto;
using namespace seahowl::fluid::aero;
using seahowl::Vector3d;

Blade::Blade(const std::shared_ptr<seahowl::elasto::BladeElasto> elasto,
             const std::shared_ptr<seahowl::aero::BladeAero> aero)
    : ComponentDynamic(elasto, aero), ComponentElastoFluid(elasto, aero), elasto(*elasto), aero(*aero) {}

void Blade::initialize_this(double time, double dt) {
    // mappings
    compute_mapping_fluid2elasto();
    compute_mapping_elasto2fluid();
    // update position of aero points
    update_positions_aero();

    spdlog::info("Initialized blade of total mass {:.4}kg with {} elasto and {} aero elements.", elasto.get_mass(),
                 elasto.discretization_fractions.size() - 1, aero.elements.size());
}

void Blade::prestep(double time, double dt) {
    // update loads on elasto part
    update_loads_elasto();
}

void Blade::poststep(double time, double dt) {
    // update position of aero points
    update_positions_aero();
}

void Blade::apply_env_model(seahowl::env::EnvModel& env_model, double time) {
    spdlog::warn("env model must be applied from Rotor instead of Blade directly.");
}

void Blade::build() {
    // build aero & elasto
    elasto.build();
    aero.build();
}

void Blade::apply_pitch_increment(double pitch_increment) {
    // pitch elasto part of blade
    elasto.apply_pitch_increment(pitch_increment);
    // update aero positions from pitched elasto blade
    update_positions_aero();
}

void Blade::set_discretization_elasto(const std::vector<double>& fractions) {
    elasto.discretization_fractions = fractions;
};

void Blade::set_discretization_aero(const std::vector<double>& fractions) {
    aero.discretization_fractions = fractions;
};

void Blade::update_positions_aero() {
    for (int ii = 0; ii < aero.nodes.size(); ii++) {
        auto& node_aero = aero.nodes[ii];

        // update position and rotation of aero elements
        int elasto_element_index = mapping_fluid2elasto_nodes[ii].index;
        double eta = mapping_fluid2elasto_nodes[ii].eta;
        auto entity = elasto.get_entity_along_blade(eta, elasto_element_index);
        node_aero.set_rotation(entity.get_rotation());
        node_aero.set_position(entity.get_position() + node_aero.get_offset_aero_absolute());
        node_aero.set_velocity(entity.get_velocity());
        node_aero.set_rotational_velocity(entity.get_rotational_velocity());
        node_aero.set_acceleration(entity.get_acceleration());
        node_aero.set_rotational_acceleration(entity.get_rotational_acceleration());
    }

    // update pitch of blade for aero
    aero.pitch = elasto.get_pitch();

    // update blade body root required by AeroDyn coupling
    auto& elasto_root = *elasto.actuator_pitch->body_worker;
    aero.body_root->set_rotation(elasto_root.get_rotation());
    aero.body_root->set_position(elasto_root.get_position());
    aero.body_root->set_velocity(elasto_root.get_velocity());
    aero.body_root->set_rotational_velocity(elasto_root.get_rotational_velocity());
    aero.body_root->set_acceleration(elasto_root.get_acceleration());
    aero.body_root->set_rotational_acceleration(elasto_root.get_rotational_acceleration());
}

void Blade::update_loads_elasto() {
    elasto.reset_loads();
    if (aero.elements.size() != mapping_fluid2elasto_elements.size()) {
        throw std::runtime_error("Blade: length of vector of elements (" + std::to_string(aero.elements.size()) +
                                 " and length of aero to elasto mapping(" +
                                 std::to_string(mapping_fluid2elasto_elements.size()) + ") do not match.");
    }
    for (int ii = 0; ii < aero.elements.size(); ii++) {
        elasto.accumulate_load_along_blade(
            aero.elements[ii].get_load(), aero.elements[ii].get_moment(), mapping_fluid2elasto_elements[ii].index,
            mapping_fluid2elasto_elements[ii].eta, aero.elements[ii].get_offset_aero_absolute());
    }
}
