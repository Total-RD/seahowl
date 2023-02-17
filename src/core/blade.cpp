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
        // add aero reference point only if airfoil properties were defined
        if (point.airfoil_properties.size() > 0) {
            aero->reference_points.push_back(BladeReferencePointAero(point));
        }
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
    std::vector<double> aero_discretization_fractions_elements;
    for (auto& element : aero->elements) {
        aero_discretization_fractions_elements.push_back(element.fraction);
    }
    mapping_aero2elasto_elements =
        get_indice_and_positions(aero_discretization_fractions_elements, elasto->discretization_fractions);
    std::vector<double> aero_discretization_fractions_nodes;
    for (auto& node : aero->nodes) {
        aero_discretization_fractions_nodes.push_back(node.properties.fraction);
    }
    mapping_aero2elasto_nodes =
        get_indice_and_positions(aero_discretization_fractions_nodes, elasto->discretization_fractions);
}

void Blade::compute_mapping_elasto2aero() {
    mapping_elasto2aero = get_indice_and_positions(elasto->discretization_fractions, aero->discretization_fractions);
}

void Blade::update_positions_aero() {
    for (int ii = 0; ii < aero->nodes.size(); ii++) {
        // update position and rotation of aero elements
        int elasto_element_index = mapping_aero2elasto_nodes[ii].index;
        auto element_elasto = elasto->elements[elasto_element_index];
        double eta = mapping_aero2elasto_nodes[ii].eta;
        auto& node_aero = aero->nodes[ii];
        elasto->evaluate_position_rotation(node_aero.coordinates, node_aero.rotation, elasto_element_index, eta);

        // add offset
        auto& offset = node_aero.properties.offset_aero;
        auto coordsys = chrono::ChCoordsys(node_aero.coordinates, node_aero.rotation);
        auto offset3D = chrono::ChVector<double>(0.0, offset.y(), -offset.x());  // assumes offset in IEC coords
        node_aero.coordinates = coordsys.TransformLocalToParent(offset3D);

        // update velocity of aero elements
        double eta_scaled = 0.5 * (eta + 1.0);
        node_aero.velocity =
            ((1.0 - eta_scaled) *
                 std::dynamic_pointer_cast<chrono::fea::ChNodeFEAxyzrot>(element_elasto->GetNodeN(0))->GetPos_dt() +
             eta_scaled *
                 std::dynamic_pointer_cast<chrono::fea::ChNodeFEAxyzrot>(element_elasto->GetNodeN(1))->GetPos_dt());
        node_aero.rot_velocity =
            ((1.0 - eta_scaled) *
                 std::dynamic_pointer_cast<chrono::fea::ChNodeFEAxyzrot>(element_elasto->GetNodeN(0))->GetWvel_par() +
             eta_scaled *
                 std::dynamic_pointer_cast<chrono::fea::ChNodeFEAxyzrot>(element_elasto->GetNodeN(1))->GetWvel_par());
        node_aero.acceleration =
            ((1.0 - eta_scaled) *
                 std::dynamic_pointer_cast<chrono::fea::ChNodeFEAxyzrot>(element_elasto->GetNodeN(0))->GetPos_dtdt() +
             eta_scaled *
                 std::dynamic_pointer_cast<chrono::fea::ChNodeFEAxyzrot>(element_elasto->GetNodeN(1))->GetPos_dtdt());
        node_aero.rot_acceleration =
            ((1.0 - eta_scaled) *
                 std::dynamic_pointer_cast<chrono::fea::ChNodeFEAxyzrot>(element_elasto->GetNodeN(0))->GetWacc_par() +
             eta_scaled *
                 std::dynamic_pointer_cast<chrono::fea::ChNodeFEAxyzrot>(element_elasto->GetNodeN(1))->GetWacc_par());
    }

    // update pitch of blade for aero
    aero->pitch = elasto->pitch;
}

void Blade::update_loads_elasto() {
    elasto->reset_loads();
    if (aero->loads.size() != mapping_aero2elasto_elements.size()) {
        throw std::runtime_error("length of vector of loads and aero to elasto mapping do not match.");
    }
    for (int ii = 0; ii < aero->loads.size(); ii++) {
        elasto->accumulate_element_load(aero->loads[ii], mapping_aero2elasto_elements[ii].index,
                                        mapping_aero2elasto_elements[ii].eta,
                                        aero->elements[ii].get_offset_aero_absolute());
    }
}