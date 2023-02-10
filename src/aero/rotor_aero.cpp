#include "seahowl/aero/rotor_aero.h"
#include <seahowl/core/utils.h>
#include <seahowl/aero/bemt.h>
#include <cmath>

using seahowl::aero::BladeAero;
using seahowl::aero::RotorAero;
using seahowl::aero::TowerAero;

RotorAero::RotorAero() {}
RotorAero::~RotorAero() {}

void RotorAero::build(std::vector<std::shared_ptr<BladeAero>> blades) {
    this->blades = blades;

    // calculate rotor radius
    radius = 0.0;
    for (int ii = 0; ii < blades.size(); ii++) {
        auto& blade = blades[ii];
        radius += (blade->discretized_points.back().coordinates - hub_position).Length();
    }
    radius /= blades.size();

    // compute blade elements related values
    compute_distances_from_tip();
    compute_distances_from_hub();
    compute_radii();
    compute_chords_solidity();
}

void RotorAero::compute_chords_solidity() {
    auto nblades = blades.size();
    for (int ii = 0; ii < blades.size(); ii++) {
        auto& blade = blades[ii];
        for (int jj = 0; jj < blade->elements.size(); jj++) {
            auto& element = blade->elements[jj];
            auto radius = (element.properties.coordinates - hub_position).Length();
            element.swept_annulus = element.length * 2 * chrono::CH_C_PI * radius;
            element.chord_solidity = nblades * element.properties.chord / (2 * chrono::CH_C_PI * radius);
            // std::cout << element.swept_annulus << " " << element.chord_solidity << std::endl;
        }
    }
}

void RotorAero::compute_distances_from_hub() {
    for (int ii = 0; ii < blades.size(); ii++) {
        auto& blade = blades[ii];
        blade->compute_distances_from_hub(hub_position, hub_radius);
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
        blade->compute_radii(hub_position);
    }
}

void RotorAero::compute_wind_loads_bemt(const WindModel& wind_model,
                                        double time,
                                        const TowerAero& tower_aero,
                                        bool tower_shadow,
                                        bool tip_loss,
                                        bool hub_loss) {
    double density = wind_model.get_density();
    for (int kk = 0; kk < 3; kk++) {
        auto& blade = blades[kk];
        auto blade_azimuth = azimuth + blade->azimuth0;
        // check that blade_azimuth is between pi and -pi
        if (blade_azimuth < -chrono::CH_C_PI || blade_azimuth > chrono::CH_C_PI) {
            blade_azimuth =
                abs(std::fmod((blade_azimuth + 3 * chrono::CH_C_PI), 2 * chrono::CH_C_PI)) - chrono::CH_C_PI;
        }
        for (int ii = 0; ii < blade->elements.size(); ii++) {
            auto& element = blade->elements[ii];
            auto& position = element.properties.coordinates;
            auto& rotation = element.properties.rotation;
            auto& velocity = element.properties.velocity;

            // get fluid relative velocity
            auto wind_velocity0 = wind_model.get_wind_velocity(position, time);
            auto wind_velocity = wind_velocity0;

            // correct wind velocity with tower shadow (if activated)
            if (tower_shadow) {
                seahowl::aero::apply_tower_shadow_effect_on_wind(wind_velocity, position, blade_azimuth, tower_aero);
            }

            auto global_velocity = wind_velocity - velocity;
            // project in disc frame
            auto local_velocity_disc = hub_rotation.RotateBack(global_velocity);

            // get global/local directions
            // pointing from hub towards nacelle
            auto local_direction_normal = chrono::ChVector<double>(0.0, 0.0, 1.0);
            auto global_direction_normal = hub_rotation.Rotate(local_direction_normal);
            // pointing from hub to element position
            auto global_direction_hub2element = (position - hub_position).GetNormalized();
            // pointing in tangential direction
            auto global_direction_tangent = (global_direction_normal % global_direction_hub2element).GetNormalized();

            // uninduced local velocity (2D)
            // frame perpendicular to rotor disc
            // x: tangential velocity (coplanar with rotor disc)
            // y: normal velocity (normal to rotor disc, pointing from hub to nacelle)
            double local_velocity_normal = (global_velocity ^ global_direction_normal);
            double local_velocity_tangent = (global_velocity ^ global_direction_tangent);
            auto local_velocity0 = chrono::ChVector2<double>(local_velocity_tangent, local_velocity_normal);

            if (local_velocity0.Length() == 0.0) {
                blade->loads[ii] = chrono::ChVector<double>(0.0, 0.0, 0.0);
            } else {
                // get induced velocity (2D) from blade element
                auto local_velocity =
                    element.get_induced_velocity_rotor(local_velocity0, blades.size(), tip_loss, hub_loss);

                // get coefficients from angle of attack
                double phi = seahowl::aero::get_phi(local_velocity);
                double alpha =
                    seahowl::aero::get_alpha_from_phi(phi, (element.pitch + element.properties.structural_twist));
                auto coefficients =
                    seahowl::aero::get_aero_coefficients_from_alpha(alpha, element.properties.airfoil_properties);

                // get drag and lift coefficients
                auto cl = coefficients.lift;
                auto cd = coefficients.drag;
                // projected to rotor local frame
                double cos_phi = cos(phi);
                double sin_phi = sin(phi);
                double cn = cl * cos_phi + cd * sin_phi;
                double ct = cl * sin_phi - cd * cos_phi;

                // calculate drag and lift force
                auto vel = local_velocity.Length();
                auto chord = element.properties.chord;
                auto length = element.length;
                auto load_n = 0.5 * density * vel * vel * chord * cn * length;
                auto load_t = 0.5 * density * vel * vel * chord * ct * length;

                // transform from local to global load
                auto load_n_global = global_direction_normal * load_n;
                auto load_t_global = global_direction_tangent * load_t;
                auto load_global = load_n_global + load_t_global;

                // store load in global frame
                blade->loads[ii] = load_global;
                blade->wind_velocities[ii] = wind_velocity0;
                blade->wind_velocities_shadowed[ii] = wind_velocity;
                blade->relative_velocities_induced[ii] =
                    global_direction_normal * local_velocity.y() + global_direction_tangent * local_velocity.x();
            }
        }
    }
}

void RotorAero::compute_wind_loads_aerodyn(float *LoadAeroDyn,
                                           const WindModel& wind_model,
                                           double time,
                                           const TowerAero& tower_aero,
                                           bool tower_shadow,
                                           bool tip_loss,
                                           bool hub_loss) {
    double density = wind_model.get_density();
    for (int kk = 0; kk < 3; kk++) {
        auto& blade = blades[kk];
        auto blade_azimuth = azimuth + blade->azimuth0;
        // check that blade_azimuth is between pi and -pi
        if (blade_azimuth < -chrono::CH_C_PI || blade_azimuth > chrono::CH_C_PI) {
            blade_azimuth =
                abs(std::fmod((blade_azimuth + 3 * chrono::CH_C_PI), 2 * chrono::CH_C_PI)) - chrono::CH_C_PI;
        }
        for (int ii = 0; ii < blade->elements.size(); ii++) {

            auto& element = blade->elements[ii];
            auto length = element.length;

            // store load in global frame
            int indp = kk * blade->elements.size() + ii;
            int pp = (kk * (blade->elements.size() + 1) + ii) * 6;
            int qq = (kk * (blade->elements.size() + 1) + ii + 1) * 6;

            auto loadx = (LoadAeroDyn[pp]   + LoadAeroDyn[qq]  ) / 2.0 * length ;
            auto loady = (LoadAeroDyn[pp+1] + LoadAeroDyn[qq+1]) / 2.0 * length ;
            auto loadz = (LoadAeroDyn[pp+2] + LoadAeroDyn[qq+2]) / 2.0 * length ;
            blade->loads[ii].Set(loadx, loady, loadz);
            
            blade->wind_velocities[ii] = 0.0;
            blade->wind_velocities_shadowed[ii] = 0.0;
            blade->relative_velocities_induced[ii] = 0.0;
                    
            
        }
    }
}