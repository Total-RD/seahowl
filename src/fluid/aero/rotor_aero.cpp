// SPDX-License-Identifier: Apache-2.0
// Copyright 2022–2026 TotalEnergies
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//

#include "seahowl/fluid/aero/rotor_aero.h"

// SEAHOWL headers
#include "seahowl/commons/utils.h"
#include "seahowl/env/env_model.h"
#include "seahowl/fluid/aero/airfoil.h"
#include "seahowl/fluid/aero/bemt.h"
#include "seahowl/fluid/aero/blade_aero.h"
#include "seahowl/fluid/aero/tower_aero.h"

// Third-party libraries
#include <spdlog/spdlog.h>

// Standard library
#include <cmath>

using namespace seahowl::fluid::aero;
using seahowl::env::EnvModel;
using seahowl::Vector3d;
using seahowl::Vector2d;
using seahowl::PI;

Vector3d project_vector_to_plane(const Vector3d& vec, const Vector3d& plane_normal) {
    auto vec_projected = (vec - (vec.dot(plane_normal)) * plane_normal);
    return vec_projected;
}

double get_vector_angle_from_plane(const Vector3d& vec, const Vector3d& plane_normal, const Vector3d& plane_tangent) {
    double angle = acos(vec.dot(plane_normal)) - seahowl::PI / 2;
    if ((plane_normal.cross(vec)).dot(plane_tangent) < 0) {
        angle = -angle;
    }
    return angle;
}

double acos_safe(double val) {
    auto tol = 1e-6;
    if (-1.0 > val && val < -(1.0 + tol)) {
        val = -1.0;
    } else if (1.0 < val && val < 1.0 + tol) {
        val = 1.0;
    }
    return acos(val);
}

seahowl::Vector2d DiskCoefficients::get_disk_coefficients_from_table(double TSR, double pitch) {
    // interpolate rotor performance for the current TSR and pitch
    double Cp = bilinear_interpolation(power_coeff, pitch_list, tsr_list, pitch, TSR);
    double Ct = bilinear_interpolation(thrust_coeff, pitch_list, tsr_list, pitch, TSR);

    seahowl::Vector2d results;
    results[0] = Cp;
    results[1] = Ct;

    return results;
}

void RotorAero::compute_disk_averaged_wind_velocity(const EnvModel& env_model, double time) {
    disk_averaged_wind_velocity = env_model.fluid_models.get_velocity(body_hub.get_position(), time);
    size_t npoints = 1;
    for (const auto& blade : blades) {
        for (const auto& node : blade->nodes) {
            node.get_position();
            disk_averaged_wind_velocity += env_model.fluid_models.get_velocity(node.get_position(), time);
            npoints += 1;
        }
    }
    disk_averaged_wind_velocity /= npoints;
}

RotorNacelleAssemblyAero::RotorNacelleAssemblyAero() {}

void RotorNacelleAssemblyAero::compute_env_loads(const EnvModel& wind_model, double time) {
    rotor->compute_disk_averaged_wind_velocity(wind_model, time);
    rotor->compute_env_loads(wind_model, time);
}

void RotorNacelleAssemblyAero::build() {
    rotor->build();
}

void RotorNacelleAssemblyAero::initialize() {
    if (!rotor) {
        std::runtime_error("Cannot initialize aero RNA without having defined and attached a rotor to it.");
    }
    rotor->initialize();
}

RotorAeroBET::RotorAeroBET() {}

void RotorAeroBET::build() {
    // build blades
    for (auto& blade : blades) {
        blade->build();
    }

    // calculate rotor radius
    radius = 0.0;
    for (int ii = 0; ii < blades.size(); ii++) {
        const auto& blade = blades[ii];
        radius += (blade->discretized_points.back().coordinates - body_hub.get_position()).norm();
    }
    radius /= blades.size();

    initialize();
}

void RotorAeroBET::initialize() {
    // compute blade elements related values
    compute_radii_distances_solidity();
}

void RotorAeroBET::compute_radii_distances_solidity() {
    auto disk_normal = body_hub.get_rotation() * Vector3d(1.0, 0.0, 0.0);
    int nblades = blades.size();

    // compute radii and distances
    radius = 0.0;
    for (const auto& blade : blades) {
        auto tip_position = blade->nodes.back().get_position();
        for (auto& node : blade->nodes) {
            node.radius = (node.get_position() - body_hub.get_position()).norm();
            node.distance_from_tip = (tip_position - node.get_position()).norm();
            node.distance_from_hub = node.radius - hub_radius;
        }
        if (blade->nodes.back().radius > radius) {
            radius = blade->nodes.back().radius;
        }
    }

    // compute chord solidities (to compute after blade radius is set)
    for (const auto& blade : blades) {
        for (auto& node : blade->nodes) {
            node.chord_solidity = nblades * node.properties.chord / (2 * PI * node.radius);
        }
    }
}

void RotorAeroBET::compute_env_loads(const EnvModel& env_model, double time) {
    compute_radii_distances_solidity();

    for (const auto& blade : blades) {
        blade->compute_env_loads(env_model, time);
    }
}

RotorAeroBEMT::RotorAeroBEMT(TowerAero& tower_ref) : tower_ref(tower_ref) {}

void RotorAeroBEMT::build() {
    RotorAeroBET::build();
}

void RotorAeroBEMT::initialize() {
    RotorAeroBET::initialize();
}

void RotorAeroBEMT::compute_env_loads(const EnvModel& env_model, double time) {
    compute_radii_distances_solidity();

    auto hub_position = body_hub.get_position();
    auto hub_rotation = body_hub.get_rotation();
    // get normal to disk
    // pointing from hub towards nacelle
    auto disk_normal = hub_rotation * Vector3d(1.0, 0.0, 0.0);

    // iterate over blades
    for (const auto& blade : blades) {
        auto blade_azimuth = azimuth + blade->azimuth0;
        // check that blade_azimuth is between pi and -pi
        if (blade_azimuth < -PI || blade_azimuth > PI) {
            blade_azimuth = abs(std::fmod((blade_azimuth + 3 * PI), 2 * PI)) - PI;
        }
        // tangent from root, to use if sweep is assumed to be through shear
        auto root_axis = blade->nodes.front().get_rotation() * Vector3d(0.0, 0.0, 1.0);
        auto disk_axis = project_vector_to_plane(root_axis, disk_normal).normalized();
        auto disk_tangent = disk_axis.cross(disk_normal).normalized();

        // iterate over blade nodes
        for (auto& node : blade->nodes) {
            // node info
            auto node_position = node.get_position();
            auto node_rotation = node.get_rotation();
            auto node_velocity = node.get_velocity();
            auto node_normal = node_rotation * Vector3d(1.0, 0.0, 0.0);
            auto node_tangent = node_rotation * Vector3d(0.0, 1.0, 0.0);
            auto node_axis = node_rotation * Vector3d(0.0, 0.0, 1.0);

            // fluid density
            double density = env_model.fluid_models.get_density(node_position, time);
            // fluid velocity
            auto wind_velocity = env_model.fluid_models.get_velocity(node_position, time);
            // correct wind velocity with tower shadow (if activated)
            if (has_tower_shadow) {
                if (blade_azimuth > PI / 2.0 || blade_azimuth < -PI / 2.0) {
                    seahowl::fluid::aero::apply_tower_shadow_effect_on_wind(wind_velocity, node_position, tower_ref);
                }
            }
            // relative velocity
            auto global_velocity = Vector3d(wind_velocity - node_velocity);

            // get normal vector of bent blade
            // assume shear along blade for sweep
            auto angle_z = get_vector_angle_from_plane(node_axis, disk_normal, disk_tangent);
            auto blade_normal_sheared = AngleAxisd(angle_z, disk_tangent) * disk_normal;
            // unsheared
            auto node_axis_projected = project_vector_to_plane(node_axis, disk_normal).normalized();
            auto blade_tangent = node_axis_projected.cross(disk_normal);
            auto blade_normal = node_axis.cross(blade_tangent);

            // make coordinate system to use for BEMT
            auto global_normal = blade_normal_sheared;
            auto global_tangent = disk_tangent;
            auto global_axis = global_normal.cross(global_tangent);

            // uninduced local velocity (2D)
            // frame perpendicular to rotor disc
            // x airfoil: tangential velocity (tangential to chord, pointing towards tail of airfoil) --> y IEC
            // y airfoil: normal velocity (normal to chord, pointing up) --> x IEC
            double local_velocity_normal0 = global_velocity.dot(global_normal);
            double local_velocity_tangent0 = global_velocity.dot(global_tangent);
            auto local_velocity0 = Vector2d(local_velocity_tangent0, local_velocity_normal0);

            double tol = 1e-6;
            if (local_velocity0.norm() < tol || (node.distance_from_tip < tol && has_tip_loss) ||
                (node.distance_from_hub < tol && has_hub_loss)) {
                node.load = Vector3d(0.0, 0.0, 0.0);
            } else {
                // get angle of airfoil (pitch + twist + torsion) from plane of bent blade
                auto angle_airfoil = get_vector_angle_from_plane(node_normal, blade_tangent, -node_axis_projected);

                // get induced velocity
                auto local_velocity = get_induced_velocity(node, local_velocity0, angle_airfoil, blades.size(),
                                                           has_tip_loss, has_hub_loss);

                // get coefficients from angle of attack
                double phi = seahowl::fluid::aero::get_phi(local_velocity);
                double alpha = seahowl::fluid::aero::get_alpha_from_phi(phi, angle_airfoil);
                auto coefficients =
                    seahowl::fluid::aero::get_aero_coefficients_from_alpha(alpha, node.properties.airfoil_properties);

                // get drag and lift coefficients
                auto cl = coefficients.lift;
                auto cd = coefficients.drag;
                auto cm = coefficients.moment;
                // projected to rotor local frame
                double cos_phi = cos(phi);
                double sin_phi = sin(phi);
                double cn = cl * cos_phi + cd * sin_phi;
                double ct = -cl * sin_phi + cd * cos_phi;

                // calculate drag and lift force
                auto vel = local_velocity.norm();
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
                    global_normal * local_velocity.y() + global_tangent * local_velocity.x();
            }
        }
    }
}

void RotorAeroDisk::initialize() {
    if (radius <= 0.0) {
        throw std::runtime_error("Disk actuator rotor cannot be initialized if radius is not set.");
    }
}

void RotorAeroDisk::compute_env_loads(const EnvModel& env_model, double time) {
    auto pos_hub = body_hub.get_position();
    auto vel_hub = body_hub.get_velocity();
    double density = env_model.fluid_models.get_density(pos_hub, time);

    auto global_velocity = Vector3d(disk_averaged_wind_velocity - vel_hub);
    // project in disc frame
    auto local_velocity_disc = (body_hub.get_rotation().inverse() * global_velocity).x();

    double RPM = (body_hub.get_rotation().inverse() * body_hub.get_rotational_velocity()).x();
    double TSR = RPM * radius / local_velocity_disc;

    auto coefficients = disk_coefficients.get_disk_coefficients_from_table(TSR, pitch_collective * 180.0 / seahowl::PI);
    // get thrust and power coefficients
    auto ct = coefficients[1];
    auto cp = coefficients[0];

    // calculate drag and lift force
    auto vel = local_velocity_disc;
    auto load_n = 0.5 * density * vel * vel * seahowl::PI * radius * radius * ct;
    auto load_t = 0.0;

    if (RPM < 0.05 && RPM >= 0.0)
        load_t = 0.5 * density * vel * vel * vel * seahowl::PI * radius * radius * cp;
    else if (RPM < 0.0)
        load_t = 0.0;
    else
        load_t = 0.5 * density * vel * vel * vel * seahowl::PI * radius * radius * cp / RPM;

    // get global/local directions
    // pointing from hub towards nacelle
    auto local_direction_normal = Vector3d(1.0, 0.0, 0.0);
    auto global_direction_normal = body_hub.get_rotation() * local_direction_normal;

    // pointing from hub to node position
    auto local_direction_tangent = Vector3d(0.0, 1.0, 0.0);
    auto global_direction_tangent = body_hub.get_rotation() * local_direction_tangent;

    // transform from local to global load
    auto load_n_global = global_direction_normal * load_n;
    auto load_t_global = global_direction_tangent * load_t;

    // auto load_global = load_n_global + load_t_global;

    hub_torque_aero = load_t;
    hub_thrust_aero = load_n;
}
