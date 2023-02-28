#include <seahowl/elasto/tower_elasto.h>
#include <seahowl/core/utils.h>

#include <memory>
#include <vector>
#include <numeric>

#include <chrono/fea/ChMesh.h>
#include <chrono/fea/ChElementBeamTaperedTimoshenko.h>

using seahowl::elasto::TowerElasto;

TowerElasto::TowerElasto() {}

void TowerElasto::build() {
    // check that enough reference points were defined to create elements (at least 2)
    if (reference_points.size() <= 2) {
        throw std::runtime_error("Not enough elasto reference points defined for blade.");
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
    discretized_points = seahowl::core::get_discretized_points(discretization_fractions, reference_points);
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
        nodes[ii]->set_properties(discretized_points[ii]);
    }

    build_elements_tapered_timoshenko();
};

void TowerElasto::build_elements_tapered_timoshenko() {
    elements.clear();
    const auto nelements = nodes.size() - 1;

    if (nelements <= 0) {
        throw std::runtime_error("Trying to build blade with no element.");
    }

    for (size_t ii = 1; ii < nelements + 1; ii++) {
        // create element
        auto element = std::make_shared<BladeElementFEA>();
        // add element to blade elements vector
        elements.push_back(element);
        // set element nodes
        element->set_nodes(nodes[ii - 1], nodes[ii]);

        // apply prebend and structural twist
        auto rotation_relative = (nodes[ii]->get_rotation() * nodes[ii - 1]->get_rotation().inverse()).normalized();
        // switch from IEC standard (Z along blade) to chrono element coordinate system (X along element)
        rotation_relative =
            Quaternion(rotation_relative[0], rotation_relative[3], rotation_relative[2], rotation_relative[1]);
        element->set_prebend(rotation_relative);
    }
}
