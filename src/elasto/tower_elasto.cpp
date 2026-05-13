// SPDX-License-Identifier: Apache-2.0
// Copyright 2022–2026 TotalEnergies
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//

#include "seahowl/elasto/tower_elasto.h"

// SEAHOWL headers
#include "seahowl/commons/utils.h"
#include "seahowl/elasto/chrono_adapters.h"
#include "seahowl/elasto/reference_point_elasto.h"

// Third-party libraries
#include <spdlog/spdlog.h>

// Standard library
#include <memory>
#include <numeric>
#include <vector>

using seahowl::elasto::TowerElasto;

TowerElasto::TowerElasto() {}

void TowerElasto::build() {
    // check that enough reference points were defined to create elements (at least 2)
    if (reference_points.size() < 2) {
        throw std::runtime_error("Not enough elasto reference points defined for tower (" +
                                 std::to_string(reference_points.size()) + ").");
    }

    // check that discretization_fractions was defined, otherwise take reference point fractions
    if (discretization_fractions.size() == 0) {
        for (int ii = 0; ii < reference_points.size(); ii++) {
            discretization_fractions.push_back(reference_points[ii].fraction);
        }
    } else if (discretization_fractions.size() == 1) {
        double npoints = discretization_fractions[0] + 1;
        double dp = 1.0 / (npoints - 1);
        discretization_fractions.clear();
        for (int ii = 0; ii < int(npoints); ii++) {
            discretization_fractions.push_back(ii * dp);
        }
    }

    // build
    discretized_points = seahowl::get_discretized_points(discretization_fractions, reference_points);
    ///@todo find better way to build ReferencePointElasto from TowerReferencePointElasto
    std::vector<ReferencePointElasto> discretized_points0;
    for (int ii = 0; ii < discretized_points.size(); ii++) {
        auto discretized_point0 = ReferencePointElasto();
        discretized_point0.coordinates = discretized_points[ii].coordinates;
        discretized_point0.fraction = discretized_points[ii].fraction;
        discretized_points0.push_back(discretized_point0);
    }
    // build nodes
    build_nodes(discretized_points0);
    // apply properties
    for (int ii = 0; ii < nodes.size(); ii++) {
        auto& node = nodes[ii];
        std::dynamic_pointer_cast<NodeElastoChrono>(node)->set_properties(discretized_points[ii]);

        // initialize added mass matrices to zero
        node->set_added_mass_matrix(Eigen::Matrix<double, 6, 6>::Zero());
    }

    build_elements_tapered_timoshenko();
};

void TowerElasto::build_elements_tapered_timoshenko() {
    elements.clear();
    const auto nelements = nodes.size() - 1;

    if (nelements <= 0) {
        throw std::runtime_error("Trying to build tower with no element.");
    }

    for (size_t ii = 1; ii < nelements + 1; ii++) {
        // create element
        auto element = std::make_shared<ElementBladeElastoChrono>();
        // add element to tower elements vector
        elements.push_back(element);
        // set element nodes
        element->set_nodes(nodes[ii - 1], nodes[ii]);
    }
}

seahowl::Vector3d TowerElasto::get_tower_base_moment() const {
    return elements.front()->get_torque(-1.0);
}

seahowl::Vector3d TowerElasto::get_tower_top_moment() const {
    return elements.back()->get_torque(1.0);
}

seahowl::Vector3d TowerElasto::get_tower_base_force() const {
    return elements.front()->get_force(-1.0);
}

seahowl::Vector3d TowerElasto::get_tower_top_force() const {
    return elements.back()->get_force(1.0);
}

void TowerElasto::reset_loads() {
    ComponentElastoFEA::reset_loads();
    for (auto& node : nodes) {
        // reinitialize added mass matrices to zero
        node->set_added_mass_matrix(Eigen::Matrix<double, 6, 6>::Zero());
    }
};
