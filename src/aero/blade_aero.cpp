#include "seahowl/aero/blade_aero.h"
#include <seahowl/aero/bemt.h>

using seahowl::aero::BladeNodeAero;
using seahowl::aero::BladeElementAero;
using seahowl::aero::BladeAero;
using seahowl::aero::get_induced_velocity;

BladeNodeAero::BladeNodeAero(BladeReferencePointAero& point) {
    properties = point;
    coordinates = point.coordinates;
    rotation = point.rotation;
    velocity = chrono::ChVector<double>(0.0, 0.0, 0.0);
    rot_velocity = chrono::ChVector<double>(0.0, 0.0, 0.0);
    acceleration = chrono::ChVector<double>(0.0, 0.0, 0.0);
    rot_acceleration = chrono::ChVector<double>(0.0, 0.0, 0.0);
    load = chrono::ChVector<double>(0.0, 0.0, 0.0);
    wind_velocity = chrono::ChVector<double>(0.0, 0.0, 0.0);
    wind_velocity_shadowed = chrono::ChVector<double>(0.0, 0.0, 0.0);
    relative_velocity_induced = chrono::ChVector<double>(0.0, 0.0, 0.0);
}

chrono::ChVector2<double> BladeNodeAero::get_induced_velocity_rotor(
    const chrono::ChVector2<double>& local_velocity_rotor0,
    double blade_pitch,
    size_t nblades,
    bool tip_loss,
    bool hub_loss) {
    return get_induced_velocity(*this, local_velocity_rotor0, blade_pitch, nblades, tip_loss, hub_loss);
}

chrono::ChVector<double> BladeNodeAero::get_offset_aero_absolute() const {
    auto& offset = properties.offset_aero;
    auto coordsys = chrono::ChCoordsys(coordinates, rotation);
    auto offset3D = chrono::ChVector<double>(0.0, offset.y(), -offset.x());  // assumes offset in IEC coords
    auto offset_absolute = coordsys.TransformLocalToParent(offset3D) - coordinates;
    return offset_absolute;
}

BladeElementAero::BladeElementAero(const BladeNodeAero& node1, const BladeNodeAero& node2)
    : node1(node1), node2(node2) {
    fraction = 0.5 * (node1.properties.fraction + node2.properties.fraction);
    length = (node1.coordinates - node2.coordinates).Length();
}

chrono::ChVector<double> BladeElementAero::get_load() const {
    return 0.5 * (node1.load + node2.load) * length;
}

chrono::ChVector<double> BladeElementAero::get_position() const {
    return 0.5 * (node1.coordinates + node2.coordinates);
}

chrono::ChQuaternion<double> BladeElementAero::get_rotation() const {
    // returning rotation of node1
    // TODO: average rotation of node1 and node2
    return node1.rotation;
}

chrono::ChVector<double> BladeElementAero::get_offset_aero_absolute() const {
    return 0.5 * (node1.get_offset_aero_absolute() + node2.get_offset_aero_absolute());
}

BladeAero::BladeAero() {}

void BladeAero::build() {
    // check that enough reference points were defined to create elements (at least 2)
    if (reference_points.size() < 2) {
        throw std::runtime_error("Not enough aero reference points defined for blade.");
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
    // nodes
    nodes.clear();
    for (int ii = 0; ii < discretized_points.size(); ii++) {
        // push empty load
        nodes.push_back(BladeNodeAero(discretized_points[ii]));
    }
    // elements
    elements.clear();
    for (int ii = 0; ii < discretized_points.size() - 1; ii++) {
        elements.push_back(BladeElementAero(nodes[ii], nodes[ii + 1]));
        loads.push_back(chrono::ChVector<double>(0.0, 0.0, 0.0));
    }

    // get distance from tip
    compute_distances_from_tip();
}

void BladeAero::compute_distances_from_tip() {
    // this is the position of the element at the tip
    auto& tip_position = discretized_points.back().coordinates;
    for (auto& node : nodes) {
        node.distance_from_tip = (node.coordinates - tip_position).Length();
    }
}

void BladeAero::compute_distances_from_hub(const chrono::ChVector<double>& hub_apex_position, double hub_radius) {
    for (auto& node : nodes) {
        node.distance_from_hub = (node.coordinates - hub_apex_position).Length() - hub_radius;
    }
}

void BladeAero::compute_radii(const chrono::ChVector<double>& hub_apex_position) {
    for (auto& node : nodes) {
        node.radius = (node.coordinates - hub_apex_position).Length();
    }
}

chrono::ChVector<double> BladeAero::get_average_wind_velocity() {
    auto average = chrono::ChVector<double>(0.0, 0.0, 0.0);
    for (auto& node : nodes) {
        average += node.wind_velocity_shadowed;
    }
    average /= nodes.size();
    return average;
}

chrono::ChVector<double> BladeAero::get_total_load() {
    auto total = chrono::ChVector<double>(0.0, 0.0, 0.0);
    for (auto& load : loads) {
        total += load;
    }
    return total;
}
