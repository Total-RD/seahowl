#include "seahowl/aero/tower_aero.h"

#include "seahowl/commons/utils.h"
#include "seahowl/commons/numerics.h"
#include "seahowl/aero/reference_point_aero.h"
#include "seahowl/env/wind_models.h"

#include <spdlog/spdlog.h>

using namespace seahowl;
using namespace seahowl::aero;
using seahowl::env::FluidModel;

TowerNodeAero::TowerNodeAero(TowerReferencePointAero& point) {
    properties = point;
    set_position(point.coordinates);
    set_velocity(Vector3d(0.0, 0.0, 0.0));
    set_acceleration(Vector3d(0.0, 0.0, 0.0));
    set_rotational_velocity(Vector3d(0.0, 0.0, 0.0));
    set_rotational_acceleration(Vector3d(0.0, 0.0, 0.0));
    load = Vector3d(0.0, 0.0, 0.0);
}

TowerElementAero::TowerElementAero(const TowerNodeAero& node1, const TowerNodeAero& node2)
    : node1(node1), node2(node2) {
    fraction = 0.5 * (node1.properties.fraction + node2.properties.fraction);
    length = (node1.get_position() - node2.get_position()).norm();
}

Vector3d TowerElementAero::get_load() const {
    return 0.5 * (node1.load + node2.load) * length;
}

Vector3d TowerElementAero::get_position() const {
    return 0.5 * (node1.get_position() + node2.get_position());
}

Quaternion TowerElementAero::get_rotation() const {
    // returning rotation of node1
    // TODO: average rotation of node1 and node2
    return node1.get_rotation();
}

TowerAero::TowerAero() {}

void TowerAero::build() {
    // check that enough reference points were defined to create elements (at least 2)
    if (reference_points.size() < 2) {
        throw std::runtime_error("Not enough aero reference points defined for tower (" +
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
    // nodes
    nodes.clear();
    for (int ii = 0; ii < discretized_points.size(); ii++) {
        // push empty load
        nodes.push_back(TowerNodeAero(discretized_points[ii]));
    }
    // elements
    elements.clear();
    loads.clear();
    for (int ii = 0; ii < discretized_points.size() - 1; ii++) {
        elements.push_back(TowerElementAero(nodes[ii], nodes[ii + 1]));
        loads.push_back(Vector3d(0.0, 0.0, 0.0));
    }
}

void TowerAero::compute_aero_loads(const FluidModel& wind_model, double time) {
    for (auto& node : nodes) {
        auto density = wind_model.get_fluid_density(node.get_position(), time);
        // get fluid relative velocity
        auto wind_velocity = wind_model.get_fluid_velocity(node.get_position(), time);
        auto velocity_relative = wind_velocity - node.get_velocity();
        auto dir = node.get_rotation() * Vector3d(0.0, 0.0, 1.0);  // tangent direction
        auto dot = velocity_relative.dot(dir);
        auto velocity_tangent = dir * dot;
        auto velocity_normal = velocity_relative - velocity_tangent;

        auto diameter = node.properties.diameter;
        auto cd = node.properties.drag_coefficient;
        auto load_drag = 0.5 * density * cd * diameter * velocity_normal.norm() * velocity_normal;

        node.load = load_drag;
    }

    for (int ii = 0; ii < elements.size(); ii++) {
        loads[ii] = elements[ii].get_load();
    }
}
