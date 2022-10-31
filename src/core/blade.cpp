#include "seahowl/core/blade.h"

#include <seahowl/elasto/reference_point_elasto.h>
#include <seahowl/elasto/blade_elasto.h>
#include <seahowl/aero/blade_aero.h>
#include <seahowl/core/utils.h>

#include <chrono/fea/ChMesh.h>
#include <chrono/physics/ChSystemSMC.h>

#include <memory>

using namespace seahowl::core;
using namespace seahowl::elasto;
using namespace seahowl::aero;

Blade::Blade() {
    elasto = std::make_shared<BladeElasto>();
    aero = std::make_shared<BladeAero>();
}

Blade::~Blade() {}

void Blade::init(double time, double dt) {
    prestep(time, dt);
    poststep(time, dt);
}

void Blade::prestep(double time, double dt) {
    // update loads on elasto part
    update_loads_elasto();
}

void Blade::poststep(double time, double dt) {
    // update position of aero points
    update_positions_aero();
}

void Blade::assemble(chrono::ChSystemSMC& system, std::shared_ptr<chrono::fea::ChMesh> mesh) {
    elasto->assemble(mesh);
}

void Blade::build() {
    // push reference points
    elasto->reference_points.clear();
    aero->reference_points.clear();
    for (auto& point : reference_points) {
        elasto->reference_points.push_back(BladeReferencePointElasto(point));
        aero->reference_points.push_back(BladeReferencePointAero(point));
    }
    // build aero & elasto
    elasto->build();
    aero->build();

    // mappings
    compute_mapping_aero2elasto();
    compute_mapping_elasto2aero();
}

void Blade::set_discretization_elasto(std::vector<double> fractions) {
    elasto->discretization_fractions = fractions;
};

void Blade::set_discretization_aero(std::vector<double> fractions) {
    aero->discretization_fractions = fractions;
};

void Blade::compute_mapping_aero2elasto() {
    // get aero element position (center) from which loads will be applied
    std::vector<double> aero_discretization_fractions;
    for (int ii = 0; ii < aero->elements.size(); ii++) {
        aero_discretization_fractions.push_back(aero->elements[ii].properties.fraction);
    }
    mapping_aero2elasto =
        get_indice_and_positions(aero_discretization_fractions, elasto->discretization_fractions);
}

void Blade::compute_mapping_elasto2aero() {
    mapping_elasto2aero =
        get_indice_and_positions(elasto->discretization_fractions, aero->discretization_fractions);
}


void Blade::update_positions_aero() {
    for (int ii = 0; ii < aero->elements.size(); ii++) {
        // update position and rotation of aero elements
        int elasto_element_index = mapping_aero2elasto[ii].index;
        double eta = mapping_aero2elasto[ii].eta;
        elasto->evaluate_position_rotation(aero->elements[ii].properties.coordinates,
                                           aero->elements[ii].properties.rotation, elasto_element_index, eta);

        // update velocity of aero elements
        aero->elements[ii].properties.velocity = 0.5 * (std::dynamic_pointer_cast<chrono::fea::ChNodeFEAxyzrot>(
                                                            elasto->elements[elasto_element_index]->GetNodeN(0))
                                                            ->GetPos_dt() +
                                                        std::dynamic_pointer_cast<chrono::fea::ChNodeFEAxyzrot>(
                                                            elasto->elements[elasto_element_index]->GetNodeN(1))
                                                            ->GetPos_dt());

        // update pitch of aero elements
        aero->elements[ii].pitch = elasto->pitch;
    }
}

void Blade::update_loads_elasto() {
    elasto->reset_loads();
    if (aero->loads.size() != mapping_aero2elasto.size()) {
        throw std::runtime_error("length of vector of loads and aero to elasto mapping do not match.");
    }
    for (int ii = 0; ii < aero->loads.size(); ii++) {
        elasto->accumulate_element_load(aero->loads[ii], mapping_aero2elasto[ii].index, mapping_aero2elasto[ii].eta);
    }
}
