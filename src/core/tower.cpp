#include "seahowl/core/tower.h"

#include "seahowl/commons/utils.h"
#include "seahowl/elasto/tower_elasto.h"
#include "seahowl/aero/tower_aero.h"

#include <spdlog/spdlog.h>

using namespace seahowl::core;
using namespace seahowl::elasto;
using namespace seahowl::aero;

Tower::Tower(TowerElasto& elasto, TowerAero& aero) : elasto(elasto), aero(aero) {}

void Tower::initialize(double time, double dt) {
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
    // get aero element position (center) from which loads will be applied
    std::vector<double> aero_discretization_fractions;
    for (int ii = 0; ii < aero.elements.size(); ii++) {
        aero_discretization_fractions.push_back(aero.elements[ii].properties.fraction);
    }
    mapping_aero2elasto = get_indice_and_positions(aero_discretization_fractions, elasto.discretization_fractions);
}

void Tower::compute_mapping_elasto2aero() {
    mapping_elasto2aero = get_indice_and_positions(elasto.discretization_fractions, aero.discretization_fractions);
}

void Tower::update_positions_aero() {
    for (int ii = 0; ii < aero.elements.size(); ii++) {
        // update position and rotation of aero elements
        int elasto_element_index = mapping_aero2elasto[ii].index;
        auto element_elasto = elasto.elements[elasto_element_index];
        double eta = mapping_aero2elasto[ii].eta;
        auto& element_aero = aero.elements[ii];
        elasto.evaluate_position_rotation(element_aero.properties.coordinates, element_aero.properties.rotation,
                                          elasto_element_index, eta);

        // update velocity of aero elements
        aero.elements[ii].properties.velocity =
            0.5 * (element_elasto->nodes[0]->get_velocity() + element_elasto->nodes[1]->get_velocity());
    }
}

void Tower::update_loads_elasto() {
    elasto.reset_loads();
    if (aero.loads.size() != mapping_aero2elasto.size()) {
        throw std::runtime_error("Tower: length of vector of loads (" + std::to_string(aero.loads.size()) +
                                 " and length of aero to elasto mapping(" + std::to_string(mapping_aero2elasto.size()) +
                                 ") do not match.");
    }
    auto offset = Vector3d(0.0, 0.0, 0.0);
    for (int ii = 0; ii < aero.loads.size(); ii++) {
        elasto.accumulate_element_load(aero.loads[ii], Vector3d(0.0, 0.0, 0.0), mapping_aero2elasto[ii].index,
                                       mapping_aero2elasto[ii].eta, offset);
    }
}
