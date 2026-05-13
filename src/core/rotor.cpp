// SPDX-License-Identifier: Apache-2.0
// Copyright 2022–2026 TotalEnergies
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//

#include "seahowl/core/rotor.h"

// SEAHOWL headers
#include "seahowl/core/blade.h"
#include "seahowl/elasto/blade_elasto.h"
#include "seahowl/elasto/rotor_elasto.h"
#include "seahowl/fluid/aero/blade_aero.h"
#include "seahowl/fluid/aero/rotor_aero.h"

// Third-party libraries
#include <spdlog/spdlog.h>

// Standard library
#include <memory>
#include <vector>

using namespace seahowl;
using namespace seahowl::core;
using namespace seahowl::elasto;
using namespace seahowl::fluid::aero;

Rotor::Rotor(std::shared_ptr<seahowl::elasto::RotorElasto> elasto, std::shared_ptr<seahowl::aero::RotorAero> aero)
    : ComponentDynamic(elasto, aero), elasto(*elasto), aero(*aero) {
    // initialize blades
    auto it_aero = aero->blades.begin();
    auto it_elasto = elasto->blades.begin();
    for (; it_aero != aero->blades.end() && it_elasto != elasto->blades.end(); ++it_aero, ++it_elasto) {
        blades.push_back(std::make_shared<Blade>(*it_elasto, *it_aero));
    }
}

void Rotor::initialize_this(double time, double dt) {
    for (auto& blade : blades) {
        blade->initialize(time, dt);
        // update initial azimuth of aero blade
        blade->aero.azimuth0 = blade->elasto.azimuth0;
    }
}

void Rotor::prestep(double time, double dt) {
    // blades
    for (auto& blade : blades) {
        blade->prestep(time, dt);
    }
}

void Rotor::poststep(double time, double dt) {
    for (auto& blade : blades) {
        blade->poststep(time, dt);
    }
}

void Rotor::build() {
    // build elasto
    elasto.build();
    // build aero
    aero.build();
}

RotorNacelleAssembly::RotorNacelleAssembly(std::shared_ptr<seahowl::elasto::RotorNacelleAssemblyElasto> elasto,
                                           std::shared_ptr<seahowl::aero::RotorNacelleAssemblyAero> aero)
    : ComponentDynamic(elasto, aero), elasto(*elasto), aero(*aero), rotor(elasto->rotor, aero->rotor) {}

void RotorNacelleAssembly::initialize_this(double time, double dt) {
    rotor.initialize(time, dt);

    update_positions_aero();
    // initialize aero variables after updating positions
    aero.initialize();

    double rotor_inertia = elasto.rotor->body_hub->get_inertia_matrix()(0, 0);
    for (const auto& blade : elasto.rotor->blades) {
        try {
            const auto& blade_fea = dynamic_cast<seahowl::elasto::BladeElastoFEA&>(*blade);
            const auto& pts = blade_fea.discretized_points;
            for (size_t i = 0; i + 1 < pts.size(); ++i) {
                // segment between i and i+1: lineic mass = mass_matrix(0,0)
                double m_per_l = 0.5 * (pts[i].mass_matrix(0, 0) + pts[i + 1].mass_matrix(0, 0));
                double z_mid = 0.5 * (pts[i].coordinates.z() + pts[i + 1].coordinates.z());
                double dl = pts[i + 1].coordinates.z() - pts[i].coordinates.z();
                rotor_inertia += m_per_l * z_mid * z_mid * dl;
            }
        } catch (const std::bad_cast&) {
            // rigid/disk blade: contribution already in hub.inertia(0,0)
        }
    }

    spdlog::info("Initialized RNA of total mass {:.4}kg., rotor inertia {:.4}kg.m^2.", elasto.get_mass(),
                 rotor_inertia);
}

void RotorNacelleAssembly::prestep(double time, double dt) {
    // rotor
    rotor.prestep(time, dt);

    // apply extra torque and thrust (if any) to hub
    elasto.rotor->body_hub->accumulate_torque_internals(Vector3d(aero.rotor->hub_torque_aero, 0, 0), true);
    elasto.rotor->body_hub->accumulate_force_internals(Vector3d(aero.rotor->hub_thrust_aero, 0, 0), true);
}

void RotorNacelleAssembly::poststep(double time, double dt) {
    // rotor
    rotor.poststep(time, dt);

    update_positions_aero();
}

void RotorNacelleAssembly::apply_env_model(seahowl::env::EnvModel& env_model, double time) {
    aero.compute_env_loads(env_model, time);
}

void RotorNacelleAssembly::update_positions_aero() {
    // pitch collective
    aero.rotor->pitch_collective = elasto.rotor->pitch_collective;
    // azimuth
    aero.rotor->azimuth = elasto.get_azimuth();
    // body_hub
    aero.rotor->body_hub.set_position(elasto.rotor->body_hub->get_position());
    aero.rotor->body_hub.set_rotation(elasto.rotor->body_hub->get_rotation());
    aero.rotor->body_hub.set_velocity(elasto.rotor->body_hub->get_velocity());
    aero.rotor->body_hub.set_acceleration(elasto.rotor->body_hub->get_acceleration());
    aero.rotor->body_hub.set_rotational_velocity(elasto.rotor->body_hub->get_rotational_velocity());
    aero.rotor->body_hub.set_rotational_acceleration(elasto.rotor->body_hub->get_rotational_acceleration());
    // body_nacelle
    aero.body_nacelle.set_position(elasto.body_nacelle->get_position());
    aero.body_nacelle.set_rotation(elasto.body_nacelle->get_rotation());
    aero.body_nacelle.set_velocity(elasto.body_nacelle->get_velocity());
    aero.body_nacelle.set_acceleration(elasto.body_nacelle->get_acceleration());
    aero.body_nacelle.set_rotational_velocity(elasto.body_nacelle->get_rotational_velocity());
    aero.body_nacelle.set_rotational_acceleration(elasto.body_nacelle->get_rotational_acceleration());
}

void RotorNacelleAssembly::build() {
    // build elasto
    elasto.build();
    // update hub position from elasto
    update_positions_aero();
    // build aero
    aero.build();
}

Vector3d project_vector_to_plane2(const Vector3d& vec, const Vector3d& plane_normal) {
    auto vec_projected = (vec - (vec.dot(plane_normal)) * plane_normal);
    return vec_projected;
}

double RotorNacelleAssembly::get_yaw_error() const {
    auto tol = 1e-6;
    if (aero.rotor->disk_averaged_wind_velocity.norm() < tol) {
        // if wind velocity is zero, return no yaw error
        return 0.0;
    }

    // disk normal vector
    auto global_z = Vector3d(0.0, 0.0, 1.0);
    auto disk_normal = elasto.rotor->body_hub->get_rotation() * Vector3d(1.0, 0.0, 0.0);
    auto disk_normal_projected = project_vector_to_plane2(disk_normal, global_z).normalized();

    // wind vector
    auto disk_wind_relative = aero.rotor->disk_averaged_wind_velocity - aero.rotor->body_hub.get_velocity();
    auto disk_wind_projected = project_vector_to_plane2(disk_wind_relative, global_z).normalized();

    auto yaw_error = atan2(disk_normal_projected.cross(disk_wind_projected).dot(global_z),
                           disk_wind_projected.dot(disk_normal_projected));
    return yaw_error;
}
