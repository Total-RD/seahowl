#include "seahowl/aero/rotor_aero.h"
#include <seahowl/aero/bemt.h>
#include <cmath>

using seahowl::aero::BladeAero;
using seahowl::aero::RotorAero;
using seahowl::aero::TowerAero;
using seahowl::Vector3d;
using seahowl::Vector2d;
using seahowl::PI;

RotorAero::RotorAero() {}

void RotorAero::build() {
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

void RotorAero::initialize() {
    // compute blade elements related values
    compute_distances_from_tip();
    compute_distances_from_hub();
    compute_radii();
    compute_chords_solidity();
}

void RotorAero::compute_chords_solidity() {
    auto nblades = blades.size();
    for (auto& blade : blades) {
        for (auto& node : blade->nodes) {
            auto radius = (node.get_position() - body_hub.get_position()).norm();
            node.chord_solidity = nblades * node.properties.chord / (2 * PI * radius);
        }
    }
}

void RotorAero::compute_distances_from_hub() {
    for (int ii = 0; ii < blades.size(); ii++) {
        auto& blade = blades[ii];
        blade->compute_distances_from_hub(body_hub.get_position(), hub_radius);
    }
}

void RotorAero::compute_distances_from_tip() {
    for (int ii = 0; ii < blades.size(); ii++) {
        auto& blade = blades[ii];
        blade->compute_distances_from_tip();
    }
}

void RotorAero::compute_radii() {
    for (int ii = 0; ii < blades.size(); ii++) {
        auto& blade = blades[ii];
        blade->compute_radii(body_hub.get_position());
    }
}

void RotorAero::compute_wind_loads_bemt(WindModel& wind_model,
                                        double time,
                                        const TowerAero& tower_aero,
                                        bool tower_shadow,
                                        bool tip_loss,
                                        bool hub_loss) {
    double density = wind_model.get_density();
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

            // get fluid relative velocity
            auto wind_velocity0 = wind_model.get_wind_velocity(position, time);
            Vector3d wind_velocity = wind_velocity0;

            // correct wind velocity with tower shadow (if activated)
            if (tower_shadow) {
                seahowl::aero::apply_tower_shadow_effect_on_wind(wind_velocity, position, blade_azimuth, tower_aero);
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
            if (local_velocity0.norm() < tol || (node.distance_from_tip < tol && tip_loss) ||
                (node.distance_from_hub < tol && hub_loss)) {
                node.load = Vector3d(0.0, 0.0, 0.0);
            } else {
                // get induced velocity (2D) from blade node
                auto local_velocity =
                    node.get_induced_velocity_rotor(local_velocity0, blade->pitch, blades.size(), tip_loss, hub_loss);

                // get coefficients from angle of attack
                double phi = seahowl::aero::get_phi(local_velocity);
                double alpha =
                    seahowl::aero::get_alpha_from_phi(phi, (blade->pitch + node.properties.structural_twist));
                auto coefficients =
                    seahowl::aero::get_aero_coefficients_from_alpha(alpha, node.properties.airfoil_properties);

                // get drag and lift coefficients
                auto cl = coefficients.lift;
                auto cd = coefficients.drag;
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

                // transform from local to global load
                auto load_n_global = global_direction_normal * load_n;
                auto load_t_global = global_direction_tangent * load_t;
                auto load_global = load_n_global + load_t_global;

                // store load in global frame
                node.load = load_global;
                node.wind_velocity = wind_velocity0;
                node.wind_velocity_shadowed = wind_velocity;
                node.relative_velocity_induced =
                    global_direction_normal * local_velocity.y() + global_direction_tangent * local_velocity.x();
            }
        }
        // update loads of blade
        for (int ii = 0; ii < blade->elements.size(); ii++) {
            blade->loads[ii] = blade->elements[ii].get_load();
        }
    }
}

#ifdef HAVE_AERODYN
void RotorAero::compute_wind_loads_aerodyn(float* LoadAeroDyn,
                                           WindModel& wind_model,
                                           double time,
                                           const TowerAero& tower_aero,
                                           bool tower_shadow,
                                           bool tip_loss,
                                           bool hub_loss) {
    double density = wind_model.get_density();
    int count_blade = -1;
    for (auto& blade : blades) {
        count_blade += 1;
        auto blade_azimuth = azimuth + blade->azimuth0;
        // check that blade_azimuth is between pi and -pi
        if (blade_azimuth < -PI || blade_azimuth > PI) {
            blade_azimuth = abs(std::fmod((blade_azimuth + 3 * PI), 2 * PI)) - PI;
        }
        int count_node = -1;
        for (auto& node : blade->nodes) {
            count_node += 1;
            // store load in global frame
            int pp = (count_blade * (blade->elements.size() + 1) + count_node) * 6;
            node.load = Vector3d(LoadAeroDyn[pp], LoadAeroDyn[pp + 1], LoadAeroDyn[pp + 2]);
        }
        // update loads of blade
        for (int ii = 0; ii < blade->elements.size(); ii++) {
            blade->loads[ii] = blade->elements[ii].get_load();
        }
    }
}
#endif
