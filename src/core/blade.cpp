#include "seahowl/core/blade.h"

#include "seahowl/commons/utils.h"
#include "seahowl/elasto/blade_elasto.h"
#include "seahowl/aero/blade_aero.h"

#include <memory>
#include <spdlog/spdlog.h>

using namespace seahowl::core;
using namespace seahowl::elasto;
using namespace seahowl::aero;
using seahowl::Vector3d;

Blade::Blade(seahowl::elasto::BladeElasto& elasto, seahowl::aero::BladeAero& aero) : elasto(elasto), aero(aero) {}

void Blade::initialize_this(double time, double dt) {
    // mappings
    compute_mapping_aero2elasto();
    compute_mapping_elasto2aero();
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

void Blade::apply_fluid_model(seahowl::env::FluidModel& fluid_model, double time) {
    spdlog::warn("Fluid model must be applied from Rotor instead of Blade directly.");
}

void Blade::build() {
    // build aero & elasto
    elasto.build();
    aero.build();
    update_positions_aero();
}

void Blade::set_discretization_elasto(std::vector<double> fractions) {
    elasto.discretization_fractions = fractions;
};

void Blade::set_discretization_aero(std::vector<double> fractions) {
    aero.discretization_fractions = fractions;
};

void Blade::compute_mapping_aero2elasto() {
    // get aero element position (center) from which loads will be applied
    std::vector<double> aero_discretization_fractions_elements;
    for (auto& element : aero.elements) {
        aero_discretization_fractions_elements.push_back(element.fraction);
    }
    mapping_aero2elasto_elements =
        get_indice_and_positions(aero_discretization_fractions_elements, elasto.discretization_fractions);
    std::vector<double> aero_discretization_fractions_nodes;
    for (auto& node : aero.nodes) {
        aero_discretization_fractions_nodes.push_back(node.properties.fraction);
    }
    mapping_aero2elasto_nodes =
        get_indice_and_positions(aero_discretization_fractions_nodes, elasto.discretization_fractions);
}

void Blade::compute_mapping_elasto2aero() {
    mapping_elasto2aero = get_indice_and_positions(elasto.discretization_fractions, aero.discretization_fractions);
}

void Blade::update_positions_aero() {
    for (int ii = 0; ii < aero.nodes.size(); ii++) {
        auto& node_aero = aero.nodes[ii];

        // update position and rotation of aero elements
        int elasto_element_index = mapping_aero2elasto_nodes[ii].index;
        double eta = mapping_aero2elasto_nodes[ii].eta;
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
    aero.body_root->set_rotation(elasto.body_root->get_rotation());
    aero.body_root->set_position(elasto.body_root->get_position());
    aero.body_root->set_velocity(elasto.body_root->get_velocity());
    aero.body_root->set_rotational_velocity(elasto.body_root->get_rotational_velocity());
    aero.body_root->set_acceleration(elasto.body_root->get_acceleration());
    aero.body_root->set_rotational_acceleration(elasto.body_root->get_rotational_acceleration());
}

void Blade::update_loads_elasto() {
    elasto.reset_loads();
    if (aero.loads.size() != mapping_aero2elasto_elements.size()) {
        throw std::runtime_error("Blade: length of vector of loads (" + std::to_string(aero.loads.size()) +
                                 " and length of aero to elasto mapping(" +
                                 std::to_string(mapping_aero2elasto_elements.size()) + ") do not match.");
    }
    for (int ii = 0; ii < aero.loads.size(); ii++) {
        elasto.accumulate_load_along_blade(aero.loads[ii], aero.moments[ii], mapping_aero2elasto_elements[ii].index,
                                           mapping_aero2elasto_elements[ii].eta,
                                           aero.elements[ii].get_offset_aero_absolute());
    }
}
