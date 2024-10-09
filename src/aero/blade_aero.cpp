#include "seahowl/aero/blade_aero.h"

#include "seahowl/commons/utils.h"
#include "seahowl/aero/reference_point_aero.h"
#include "seahowl/aero/bemt.h"

#include <spdlog/spdlog.h>

using namespace seahowl;
using namespace seahowl::aero;

BladeNodeAero::BladeNodeAero(BladeReferencePointAero& point) {
    properties = point;
    set_position(point.coordinates);
    set_velocity(Vector3d(0.0, 0.0, 0.0));
    set_acceleration(Vector3d(0.0, 0.0, 0.0));
    set_rotational_velocity(Vector3d(0.0, 0.0, 0.0));
    set_rotational_acceleration(Vector3d(0.0, 0.0, 0.0));
    load = Vector3d(0.0, 0.0, 0.0);
    wind_velocity = Vector3d(0.0, 0.0, 0.0);
    wind_velocity_shadowed = Vector3d(0.0, 0.0, 0.0);
    relative_velocity_induced = Vector3d(0.0, 0.0, 0.0);
}

Vector3d BladeNodeAero::get_offset_aero_absolute() const {
    auto& offset = properties.offset_aero;
    auto offset3D = Vector3d(offset.x(), offset.y(), 0.0);
    // apply rotation of node (includes twist + pitch)
    auto offset_absolute = rotation * offset3D;
    return offset_absolute;
}

BladeElementAero::BladeElementAero(const BladeNodeAero& node1, const BladeNodeAero& node2)
    : node1(node1), node2(node2) {
    fraction = 0.5 * (node1.properties.fraction + node2.properties.fraction);
    length = (node1.get_position() - node2.get_position()).norm();
}

Vector3d BladeElementAero::get_load() const {
    return 0.5 * (node1.load + node2.load) * length;
}

Vector3d BladeElementAero::get_moment() const {
    return 0.5 * (node1.moment + node2.moment) * length;
}

Vector3d BladeElementAero::get_position() const {
    return 0.5 * (node1.get_position() + node2.get_position());
}

Quaternion BladeElementAero::get_rotation() const {
    // returning rotation of node1
    // TODO: average rotation of node1 and node2
    return node1.get_rotation();
}

Vector3d BladeElementAero::get_offset_aero_absolute() const {
    return 0.5 * (node1.get_offset_aero_absolute() + node2.get_offset_aero_absolute());
}

BladeAero::BladeAero() {
    body_root = std::make_unique<EntityDynamicEigen>();
}

void BladeAero::build() {
    // check that enough reference points were defined to create elements (at least 2)
    if (reference_points.size() < 2) {
        throw std::runtime_error("Not enough aero reference points defined for blade (" +
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
        nodes.push_back(BladeNodeAero(discretized_points[ii]));
    }
    // elements
    elements.clear();
    loads.clear();
    moments.clear();
    for (int ii = 0; ii < discretized_points.size() - 1; ii++) {
        elements.push_back(BladeElementAero(nodes[ii], nodes[ii + 1]));
        loads.push_back(Vector3d(0.0, 0.0, 0.0));
        moments.push_back(Vector3d(0.0, 0.0, 0.0));
    }

    // get distance from tip
    compute_distances_from_tip();
}

void BladeAero::compute_distances_from_tip() {
    // this is the position of the element at the tip
    auto tip_position = nodes.back().get_position();
    for (auto& node : nodes) {
        node.distance_from_tip = (node.get_position() - tip_position).norm();
    }
}

void BladeAero::compute_distances_from_hub(const Vector3d& hub_apex_position, double hub_radius) {
    for (auto& node : nodes) {
        node.distance_from_hub = (node.get_position() - hub_apex_position).norm() - hub_radius;
    }
}

void BladeAero::compute_radii(const Vector3d& hub_apex_position) {
    for (auto& node : nodes) {
        node.radius = (node.get_position() - hub_apex_position).norm();
    }
}

Vector3d BladeAero::get_average_wind_velocity() {
    auto average = Vector3d(0.0, 0.0, 0.0);
    for (auto& node : nodes) {
        average += node.wind_velocity_shadowed;
    }
    average /= nodes.size();
    return average;
}

Vector3d BladeAero::get_total_load() {
    auto total = Vector3d(0.0, 0.0, 0.0);
    for (auto& load : loads) {
        total += load;
    }
    return total;
}
