#include "seahowl/core/blade.h"

#include <seahowl/elasto/reference_point_elasto.h>
#include <seahowl/elasto/blade_elasto.h>
#include <seahowl/aero/blade_aero.h>
#include <seahowl/commons/utils.h>

#include <memory>

using namespace seahowl::core;
using namespace seahowl::elasto;
using namespace seahowl::aero;
using seahowl::Vector3d;

Blade::Blade() {
    elasto = std::make_shared<BladeElasto>();
    aero = std::make_shared<BladeAero>();
}

void Blade::init(double time, double dt) {
    // mappings
    compute_mapping_aero2elasto();
    compute_mapping_elasto2aero();
    // update position of aero points
    update_positions_aero();
}

void Blade::prestep(double time, double dt) {
    // update loads on elasto part
    update_loads_elasto();
}

void Blade::poststep(double time, double dt) {
    // update position of aero points
    update_positions_aero();
}

void Blade::assemble(std::shared_ptr<MeshElasto> mesh) {
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
        Vector3d new_position;
        Quaternion new_rotation;
        elasto->evaluate_position_rotation(new_position, new_rotation, elasto_element_index, eta);
        node_aero.set_position(new_position);
        node_aero.set_rotation(new_rotation);

        // add offset
        auto& offset = node_aero.properties.offset_aero;
        auto offset3D = Vector3d(0.0, offset.y(), -offset.x());  // assumes offset in IEC coords
        node_aero.set_position(node_aero.get_position() + node_aero.get_rotation() * offset3D);

        // update properties of aero nodes
        double weight1 = 0.5 * fabs(eta - 1.0);
        double weight2 = 0.5 * fabs(eta + 1.0);
        auto node1 = element_elasto->nodes0[0];
        auto node2 = element_elasto->nodes0[1];
        node_aero.set_velocity(weight1 * node1->get_velocity() + weight2 * node2->get_velocity());
        node_aero.set_rotational_velocity(weight1 * node1->get_rotational_velocity() +
                                          weight2 * node2->get_rotational_velocity());
        node_aero.set_acceleration(weight1 * node1->get_acceleration() + weight2 * node2->get_acceleration());
        node_aero.set_rotational_acceleration(weight1 * node1->get_rotational_acceleration() +
                                              weight2 * node2->get_rotational_acceleration());
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
