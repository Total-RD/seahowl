#pragma once

#include <seahowl/elasto/mooring_elasto.h>
#include <seahowl/core/utils.h>

#include <chrono/fea/ChElementBeamEuler.h>

using namespace seahowl::elasto;

MooringElasto::MooringElasto() {}

MooringElasto::~MooringElasto() {}

void MooringElasto::build() {
    // check that enough reference points were defined to create elements (at least 2)
    if (reference_points.size() <= 2) {
        throw std::runtime_error("Not enough elasto reference points defined for blade.");
    }

    // check that discretization_fractions was defined, otherwise take reference point fractions
    if (discretization_fractions.size() == 0) {
        for (int ii = 0; ii < reference_points.size(); ii++) {
            discretization_fractions.push_back(reference_points[ii].fraction);
        }
    }

    // build
    discretized_points = seahowl::core::get_discretized_points(discretization_fractions, reference_points);
    build_nodes();
    build_elements_euler();
}

void MooringElasto::build_nodes() {
    nodes.clear();
    const auto nnodes = discretized_points.size();
    for (size_t ii = 0; ii < nnodes; ii++) {
        auto& discretized_point = discretized_points[ii];
        auto& node_pos = discretized_point.coordinates;

        // get node coordinate system
        chrono::ChVector<> node_axis;
        chrono::ChMatrix33<> node_rotation;
        if (ii == 0) {
            node_axis = (discretized_points[ii + 1].coordinates - node_pos).GetNormalized();
            node_rotation.Set_A_Xdir(node_axis, chrono::VECT_Y);
        } else if (ii == nnodes - 1) {
            node_axis = (node_pos - discretized_points[ii - 1].coordinates).GetNormalized();
            node_rotation.Set_A_Xdir(node_axis, chrono::VECT_Y);
        } else {
            node_axis =
                (discretized_points[ii + 1].coordinates - discretized_points[ii - 1].coordinates).GetNormalized();
            node_rotation.Set_A_Xdir(node_axis, chrono::VECT_Y);
        }
        auto node_frame = chrono::ChFrame<>(node_pos, node_rotation);

        // make node
        auto node = chrono_types::make_shared<chrono::fea::ChNodeFEAxyzrot>(node_frame);
        // add node to mooring nodes vector
        nodes.push_back(node);
    }
}

void MooringElasto::build_elements_euler() {
    elements.clear();
    const auto nelements = nodes.size() - 1;

    if (nelements <= 0) {
        throw std::runtime_error("Trying to build blade with no element.");
    }

    // make first section for tapered section
    auto section = chrono_types::make_shared<chrono::fea::ChBeamSectionEulerAdvanced>();

    // make it a circular section
    section->SetAsCircularSection(diameter);
    // material properties
    section->SetDensity(density);
    // axial
    section->SetYoungModulus(young_modulus);
    // damping
    //section->SetBeamRaleyghDamping(discretized_point.damping_coefficients);

    for (size_t ii = 1; ii < nelements + 1; ii++) {
        // create element
        auto element = chrono_types::make_shared<chrono::fea::ChElementBeamEuler>();
        // add element to blade elements vector
        elements.push_back(element);
        // set element nodes
        element->SetNodes(nodes[ii - 1], nodes[ii]);
        // set first section for tapered section
        element->SetSection(section);
    }
}
