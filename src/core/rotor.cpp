#include "seahowl/core/rotor.h"

#include "seahowl/core/blade.h"
#include "seahowl/elasto/blade_elasto.h"
#include "seahowl/elasto/rotor_elasto.h"
#include "seahowl/aero/blade_aero.h"
#include "seahowl/aero/rotor_aero.h"

#include <memory>
#include <vector>
#include <spdlog/spdlog.h>

using namespace seahowl;
using namespace seahowl::core;
using namespace seahowl::elasto;
using namespace seahowl::aero;

RotorNacelleAssembly::RotorNacelleAssembly(std::shared_ptr<seahowl::elasto::RotorNacelleAssemblyElasto> elasto,
                                           std::shared_ptr<seahowl::aero::RotorNacelleAssemblyAero> aero)
    : ComponentDynamic(elasto, aero), elasto(*elasto), aero(*aero) {}

void RotorNacelleAssembly::initialize_this(double time, double dt) {
    for (auto& blade : blades) {
        blade->initialize(time, dt);
        // update initial azimuth of aero blade
        blade->aero.azimuth0 = blade->elasto.azimuth0;
    }
    update_positions_aero();
    // initialize aero variables after updating positions
    aero.initialize();

    spdlog::info("Initialized RNA of total mass {:.4}kg.", elasto.get_mass());
}

void RotorNacelleAssembly::prestep(double time, double dt) {
    // blades
    for (auto& blade : blades) {
        blade->prestep(time, dt);
    }

    // apply extra torque and thrust (if any) to hub
    elasto.rotor->body_hub->accumulate_torque_internals(Vector3d(aero.rotor->hub_torque_aero, 0, 0), true);
    elasto.rotor->body_hub->accumulate_force_internals(Vector3d(aero.rotor->hub_thrust_aero, 0, 0), true);
}

void RotorNacelleAssembly::poststep(double time, double dt) {
    for (auto& blade : blades) {
        blade->poststep(time, dt);
    }
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
