// SPDX-License-Identifier: Apache-2.0
// Copyright 2022–2026 TotalEnergies
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//

#include "seahowl/fluid/aero/blade_aero.h"

// SEAHOWL headers
#include "seahowl/commons/utils.h"
#include "seahowl/env/env_model.h"
#include "seahowl/fluid/aero/bemt.h"
#include "seahowl/fluid/aero/reference_point_aero.h"

// Third-party libraries
#include <spdlog/spdlog.h>

using namespace seahowl;
using namespace seahowl::fluid::aero;

BladeNodeAero::BladeNodeAero(const BladeReferencePointAero& point) : properties(point) {
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

BladeAero::BladeAero() : body_root(std::make_unique<EntityDynamicEigen>()) {}

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
    for (int ii = 0; ii < discretized_points.size() - 1; ii++) {
        elements.push_back(BladeElementAero(nodes[ii], nodes[ii + 1]));
    }

    // get distance from tip
    compute_distances_from_tip();
}

void BladeAero::compute_env_loads(const env::EnvModel& env_model, double time) {
    // iterate over blade nodes
    for (auto& node : nodes) {
        // node info
        auto node_position = node.get_position();
        auto node_rotation = node.get_rotation();
        auto node_velocity = node.get_velocity();

        // fluid density
        double density = env_model.fluid_models.get_density(node_position, time);
        // fluid velocity
        auto wind_velocity = env_model.fluid_models.get_velocity(node_position, time);
        // relative velocity
        auto global_velocity = Vector3d(wind_velocity - node_velocity);

        // axis of node in global frame
        auto global_normal = node_rotation * Vector3d(1.0, 0.0, 0.0);
        auto global_tangent = node_rotation * Vector3d(0.0, 1.0, 0.0);
        auto global_axis = node_rotation * Vector3d(0.0, 0.0, 1.0);

        // uninduced local velocity (2D)
        // frame perpendicular to rotor disc
        // x airfoil: tangential velocity (tangential to chord, pointing towards tail of airfoil) --> y IEC
        // y airfoil: normal velocity (normal to chord, pointing up) --> x IEC
        double local_velocity_normal0 = global_velocity.dot(global_normal);
        double local_velocity_tangent0 = global_velocity.dot(global_tangent);
        auto local_velocity_airfoil = Vector2d(local_velocity_tangent0, local_velocity_normal0);

        double tol = 1e-6;
        if (local_velocity_airfoil.norm() < tol) {
            node.load = Vector3d(0.0, 0.0, 0.0);
        } else {
            // get angle of airfoil (pitch + twist + torsion) from plane of bent blade
            // auto angle_airfoil = get_vector_angle_from_plane(node_normal, blade_tangent, -node_axis_projected);
            auto angle_airfoil = 0.0;

            // get coefficients from angle of attack
            double phi = seahowl::aero::get_phi(local_velocity_airfoil);
            double alpha = seahowl::aero::get_alpha_from_phi(phi, angle_airfoil);
            auto coefficients =
                seahowl::fluid::aero::get_aero_coefficients_from_alpha(alpha, node.properties.airfoil_properties);

            // get drag and lift coefficients
            auto cl = coefficients.lift;
            auto cd = coefficients.drag;
            auto cm = coefficients.moment;
            // projected to local frame
            double cos_phi = cos(phi);
            double sin_phi = sin(phi);
            double cn = cl * cos_phi + cd * sin_phi;
            double ct = -cl * sin_phi + cd * cos_phi;

            // calculate drag and lift force
            auto vel = local_velocity_airfoil.norm();
            auto chord = node.properties.chord;
            auto load_n = 0.5 * density * vel * vel * chord * cn;
            auto load_t = 0.5 * density * vel * vel * chord * ct;
            auto moment = 0.5 * density * vel * vel * chord * chord * cm;

            // transform from local to global load
            node.load = global_normal * load_n + global_tangent * load_t;
            node.moment = global_axis * moment;

            // store info about fluid velocity
            node.wind_velocity = wind_velocity;
            node.wind_velocity_shadowed = wind_velocity;
            node.relative_velocity_induced =
                global_normal * local_velocity_airfoil.y() + global_tangent * local_velocity_airfoil.x();
        }
    }
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
    for (const auto& node : nodes) {
        average += node.wind_velocity_shadowed;
    }
    average /= nodes.size();
    return average;
}

Vector3d BladeAero::get_total_load() {
    auto total = Vector3d(0.0, 0.0, 0.0);
    for (const auto& element : elements) {
        total += element.get_load();
    }
    return total;
}
