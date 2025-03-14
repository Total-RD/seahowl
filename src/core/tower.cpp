#include "seahowl/core/tower.h"

#include "seahowl/commons/utils.h"
#include "seahowl/elasto/tower_elasto.h"
#include "seahowl/aero/tower_aero.h"

#include <spdlog/spdlog.h>

using namespace seahowl::core;
using namespace seahowl::elasto;
using namespace seahowl::aero;

Tower::Tower(TowerElasto& elasto, TowerAero& aero) : elasto(elasto), aero(aero) {}

void Tower::initialize_this(double time, double dt) {
    // mappings
    compute_mapping_aero2elasto();
    compute_mapping_elasto2aero();
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

void Tower::set_discretization_elasto(std::vector<double> fractions) {
    elasto.discretization_fractions = fractions;
}

void Tower::set_discretization_aero(std::vector<double> fractions) {
    aero.discretization_fractions = fractions;
}

void Tower::compute_mapping_aero2elasto() {
    mapping_aero2elasto_nodes =
        get_indice_and_positions(aero.discretization_fractions, elasto.discretization_fractions);

    // get aero element position (center) from which loads will be applied
    if (aero.elements.size() != aero.discretization_fractions.size() - 1) {
        throw std::runtime_error("There are " + std::to_string(aero.elements.size()) + " elements for " +
                                 std::to_string(aero.discretization_fractions.size() - 1) +
                                 "discretization fractions (" + std::to_string(aero.nodes.size()) + " nodes).");
    }
    std::vector<double> aero_discretization_fractions_elements;
    for (int ii = 0; ii < aero.elements.size(); ii++) {
        auto element_fraction = 0.5 * (aero.discretization_fractions[ii] + aero.discretization_fractions[ii + 1]);
        aero_discretization_fractions_elements.push_back(element_fraction);
    }
    mapping_aero2elasto_elements =
        get_indice_and_positions(aero_discretization_fractions_elements, elasto.discretization_fractions);
}

void Tower::compute_mapping_elasto2aero() {
    mapping_elasto2aero = get_indice_and_positions(elasto.discretization_fractions, aero.discretization_fractions);
}

void Tower::update_positions_aero() {
    for (int ii = 0; ii < aero.nodes.size(); ii++) {
        auto& node_aero = aero.nodes[ii];

        // update position and rotation of aero elements
        int elasto_element_index = mapping_aero2elasto_nodes[ii].index;
        double eta = mapping_aero2elasto_nodes[ii].eta;
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
    if (aero.loads.size() != mapping_aero2elasto_elements.size()) {
        throw std::runtime_error("Tower: length of vector of loads (" + std::to_string(aero.loads.size()) +
                                 " and length of aero to elasto mapping(" +
                                 std::to_string(mapping_aero2elasto_elements.size()) + ") do not match.");
    }
    auto offset = Vector3d(0.0, 0.0, 0.0);
    for (int ii = 0; ii < aero.loads.size(); ii++) {
        elasto.accumulate_element_load(aero.loads_noacc[ii], Vector3d(0.0, 0.0, 0.0),
                                       mapping_aero2elasto_elements[ii].index, mapping_aero2elasto_elements[ii].eta,
                                       offset);
        elasto.accumulate_mass_matrix(aero.added_mass_matrices[ii], mapping_aero2elasto_elements[ii].index,
                                      mapping_aero2elasto_elements[ii].eta);
    }
}
