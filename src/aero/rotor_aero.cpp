#include "seahowl/aero/rotor_aero.h"

#include "seahowl/aero/blade_aero.h"
#include "seahowl/aero/tower_aero.h"
#include "seahowl/aero/airfoil.h"
#include "seahowl/aero/bemt.h"
#include "seahowl/commons/utils.h"
#include "seahowl/env/wind_models.h"

#include <cmath>
#include <spdlog/spdlog.h>

using namespace seahowl::aero;
using seahowl::env::FluidModel;
using seahowl::Vector3d;
using seahowl::Vector2d;
using seahowl::PI;

seahowl::Vector2d DiskCoefficients::get_disk_coefficients_from_table(double TSR, double pitch) {
    // interpolate rotor performance for the current TSR and pitch
    double Cp = bilinear_interpolation(power_coeff, pitch_list, tsr_list, pitch, TSR);
    double Ct = bilinear_interpolation(thrust_coeff, pitch_list, tsr_list, pitch, TSR);

    seahowl::Vector2d results;
    results[0] = Cp;
    results[1] = Ct;

    return results;
}

RotorNacelleAssemblyAero::RotorNacelleAssemblyAero() {}

void RotorNacelleAssemblyAero::build() {
    rotor->build();
}

void RotorNacelleAssemblyAero::initialize() {
    if (!rotor) {
        std::runtime_error("Cannot initialize aero RNA without having defined and attached a rotor to it.");
    }
    rotor->initialize();
}

RotorAeroBEMT::RotorAeroBEMT(TowerAero& tower_ref) : tower_ref(tower_ref) {}

void RotorAeroBEMT::build() {
    // build blades
    for (auto& blade : blades) {
        blade->build();
    }

    // calculate rotor radius
    radius = 0.0;
    for (int ii = 0; ii < blades.size(); ii++) {
        auto& blade = blades[ii];
        radius += (blade->discretized_points.back().coordinates - body_hub.get_position()).norm();
    }
    radius /= blades.size();

    initialize();
}

void RotorAeroBEMT::initialize() {
    // compute blade elements related values
    compute_distances_from_tip();
    compute_distances_from_hub();
    compute_radii();
    compute_chords_solidity();
}

void RotorAeroBEMT::compute_chords_solidity() {
    auto nblades = blades.size();
    for (auto& blade : blades) {
        for (auto& node : blade->nodes) {
            auto radius = (node.get_position() - body_hub.get_position()).norm();
            node.chord_solidity = nblades * node.properties.chord / (2 * PI * radius);
        }
    }
}

void RotorAeroBEMT::compute_distances_from_hub() {
    for (int ii = 0; ii < blades.size(); ii++) {
        auto& blade = blades[ii];
        blade->compute_distances_from_hub(body_hub.get_position(), hub_radius);
    }
}

void RotorAeroBEMT::compute_distances_from_tip() {
    for (int ii = 0; ii < blades.size(); ii++) {
        auto& blade = blades[ii];
        blade->compute_distances_from_tip();
    }
}

void RotorAeroBEMT::compute_radii() {
    for (int ii = 0; ii < blades.size(); ii++) {
        auto& blade = blades[ii];
        blade->compute_radii(body_hub.get_position());
    }
}

void RotorAeroBEMT::compute_aero_loads(const FluidModel& wind_model, double time) {
    for (auto& blade : blades) {
        auto blade_azimuth = azimuth + blade->azimuth0;
        // check that blade_azimuth is between pi and -pi
        if (blade_azimuth < -PI || blade_azimuth > PI) {
            blade_azimuth = abs(std::fmod((blade_azimuth + 3 * PI), 2 * PI)) - PI;
        }
        int count = -1;
        for (auto& node : blade->nodes) {
            count += 1;
            auto position = node.get_position();
            auto rotation = node.get_rotation();
            auto velocity = node.get_velocity();

            double density = wind_model.get_fluid_density(position, time);

            // get fluid relative velocity
            auto wind_velocity0 = wind_model.get_fluid_velocity(position, time);
            Vector3d wind_velocity = wind_velocity0;

            // correct wind velocity with tower shadow (if activated)
            if (has_tower_shadow) {
                if (blade_azimuth > PI / 2.0 || blade_azimuth < -PI / 2.0) {
                    seahowl::aero::apply_tower_shadow_effect_on_wind(wind_velocity, position, tower_ref);
                }
            }

            auto global_velocity = Vector3d(wind_velocity - velocity);
            // project in disc frame
            auto local_velocity_disc = body_hub.get_rotation().inverse() * global_velocity;

            // get global/local directions
            // pointing from hub towards nacelle
            auto local_direction_normal = Vector3d(1.0, 0.0, 0.0);
            auto global_direction_normal = body_hub.get_rotation() * local_direction_normal;

            // pointing from hub to node position
            auto global_direction_hub2node = (position - body_hub.get_position()).normalized();
            // pointing in tangential direction
            auto global_direction_tangent = global_direction_normal.cross(global_direction_hub2node).normalized();

            // uninduced local velocity (2D)
            // frame perpendicular to rotor disc
            // x: tangential velocity (coplanar with rotor disc)
            // y: normal velocity (normal to rotor disc, pointing from hub to nacelle)
            double local_velocity_normal = global_velocity.dot(global_direction_normal);
            double local_velocity_tangent = global_velocity.dot(global_direction_tangent);
            auto local_velocity0 = Vector2d(local_velocity_tangent, local_velocity_normal);

            double tol = 1e-6;
            if (local_velocity0.norm() < tol || (node.distance_from_tip < tol && has_tip_loss) ||
                (node.distance_from_hub < tol && has_hub_loss)) {
                node.load = Vector3d(0.0, 0.0, 0.0);
            } else {
                // get airfoil angle with rotor plane
                auto airfoil_direction = node.get_rotation() * Vector3d(0.0, -1.0, 0.0);
                double airfoil_angle = acos(airfoil_direction.dot(global_direction_normal)) - seahowl::PI / 2;
                // check direction of airfoil axis
                if (airfoil_direction.dot(global_direction_tangent) < 0) {
                    airfoil_angle = seahowl::PI - airfoil_angle;
                }

                // get induced velocity
                auto local_velocity = get_induced_velocity(node, local_velocity0, airfoil_angle, blades.size(),
                                                           has_tip_loss, has_hub_loss);

                // get coefficients from angle of attack
                double phi = seahowl::aero::get_phi(local_velocity);
                double alpha = seahowl::aero::get_alpha_from_phi(phi, airfoil_angle);
                auto coefficients =
                    seahowl::aero::get_aero_coefficients_from_alpha(alpha, node.properties.airfoil_properties);

                // get drag and lift coefficients
                auto cl = coefficients.lift;
                auto cd = coefficients.drag;
                auto cm = coefficients.moment;
                // projected to rotor local frame
                double cos_phi = cos(phi);
                double sin_phi = sin(phi);
                double cn = cl * cos_phi + cd * sin_phi;
                double ct = cl * sin_phi - cd * cos_phi;

                // calculate drag and lift force
                auto vel = local_velocity.norm();
                auto chord = node.properties.chord;
                auto load_n = 0.5 * density * vel * vel * chord * cn;
                auto load_t = 0.5 * density * vel * vel * chord * ct;
                auto moment = 0.5 * density * vel * vel * chord * chord * cm;

                // transform from local to global load
                auto load_n_global = global_direction_normal * load_n;
                auto load_t_global = global_direction_tangent * load_t;
                auto load_global = load_n_global + load_t_global;
                auto moment_global = global_direction_hub2node * moment;

                // store load in global frame
                node.load = load_global;
                node.wind_velocity = wind_velocity0;
                node.wind_velocity_shadowed = wind_velocity;
                node.relative_velocity_induced =
                    global_direction_normal * local_velocity.y() + global_direction_tangent * local_velocity.x();
                // store moment in local frame
                node.moment = moment_global;
            }
        }
        // update loads of blade
        for (int ii = 0; ii < blade->elements.size(); ii++) {
            blade->loads[ii] = blade->elements[ii].get_load();
            blade->moments[ii] = blade->elements[ii].get_moment();
        }
    }
}

void RotorAeroDisk::initialize() {
    if (radius <= 0.0) {
        throw std::runtime_error("Disk actuator rotor cannot be initialized if radius is not set.");
    }
}

void RotorAeroDisk::compute_aero_loads(const FluidModel& wind_model, double time) {
    auto pos_hub = body_hub.get_position();
    auto vel_hub = body_hub.get_velocity();
    double density = wind_model.get_fluid_density(pos_hub, time);
    // get fluid relative velocity
    auto wind_velocity = wind_model.get_fluid_velocity(pos_hub, time);

    auto global_velocity = Vector3d(wind_velocity - vel_hub);
    // project in disc frame
    auto local_velocity_disc = (body_hub.get_rotation().inverse() * global_velocity).x();

    double RPM = (body_hub.get_rotation().inverse() * body_hub.get_rotational_velocity()).x();
    double TSR = RPM * radius / local_velocity_disc;

    auto coefficients = disk_coefficients.get_disk_coefficients_from_table(pitch_collective * 180.0 / seahowl::PI, TSR);
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
