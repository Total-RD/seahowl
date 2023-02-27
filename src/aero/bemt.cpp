#include <seahowl/aero/bemt.h>

#include <seahowl/aero/airfoil.h>
#include <seahowl/aero/blade_aero.h>
#include <seahowl/aero/tower_aero.h>
#include <seahowl/core/utils.h>

using seahowl::Vector2d;
using seahowl::Vector3d;
using seahowl::PI;

double seahowl::aero::get_phi(const Vector2d& fluid_velocity) {
    double phi = atan2(fluid_velocity.y(), -fluid_velocity.x());
    return phi;
}

double seahowl::aero::get_alpha_from_phi(const double phi, const double pitch) {
    double alpha = phi - pitch;
    // check that alpha is still in range
    if (alpha < -PI || alpha > PI) {
        alpha = fabs(std::fmod((alpha + 3 * PI), 2 * PI)) - PI;
    }
    return alpha;
}

double seahowl::aero::get_alpha(const Vector2d& fluid_velocity, const double pitch) {
    double phi = get_phi(fluid_velocity);
    double alpha = get_alpha_from_phi(phi, pitch);
    return alpha;
}

seahowl::aero::AirfoilCoefficients seahowl::aero::get_aero_coefficients_from_alpha(
    const double alpha,
    std::vector<seahowl::aero::AirfoilProperties>& airfoil_properties) {
    // get coefficients from angle of attack
    auto coefficients = airfoil_properties[0].find_coefficients(alpha * 180 / PI);
    return coefficients;
}

Vector2d seahowl::aero::get_induced_velocity(seahowl::aero::BladeNodeAero& node,
                                             const Vector2d& local_velocity_rotor0,
                                             const double blade_pitch,
                                             const size_t nblades,
                                             const bool tip_loss,
                                             const bool hub_loss) {
    // local_velocity is in local element frame
    Vector2d local_velocity;
    Vector2d local_velocity_rotor;
    double pitch_twist = blade_pitch + node.properties.structural_twist;

    double tol_rel = 1e-3;
    double tol_abs = 1e-3;
    int max_iter = 100;
    double alpha = -1000.0;
    double alpha_previous;
    auto aa = node.induction_factor_axial;
    auto ap = node.induction_factor_tangential;
    // limits
    double aa_max = 1.0;
    double aa_min = -1.0;
    double ap_max = 1.5;
    double ap_min = -1.0;
    for (int ii = 1; ii <= max_iter; ii++) {
        // store previous alpha
        alpha_previous = alpha;
        // store previous induction factors
        auto aa_previous = aa;
        auto ap_previous = ap;

        // local velocity updated with induction factors
        local_velocity_rotor = Vector2d(local_velocity_rotor0.x() * (1.0 + ap), local_velocity_rotor0.y() * (1.0 - aa));

        // get coefficients from angle of attack
        double phi = seahowl::aero::get_phi(local_velocity_rotor);
        alpha = seahowl::aero::get_alpha_from_phi(phi, (blade_pitch + node.properties.structural_twist));
        auto coefficients = seahowl::aero::get_aero_coefficients_from_alpha(alpha, node.properties.airfoil_properties);

        // get drag and lift coefficients
        auto cl = coefficients.lift;
        auto cd = coefficients.drag;
        // projected to rotor local frame
        double cos_phi = cos(phi);
        double sin_phi = sin(phi);
        double cn = cl * cos_phi + cd * sin_phi;
        double ct = cl * sin_phi - cd * cos_phi;

        // losses
        double loss_factor = 1.0;
        if (tip_loss) {
            // Prandtl's approximation for tip-loss factor=
            loss_factor *=
                (2.0 / PI) * acos(exp(nblades * (-node.distance_from_tip) / (2.0 * node.radius * fabs(sin_phi))));
        }
        if (hub_loss) {
            // hub loss
            double hub_radius = (node.radius - node.distance_from_hub);
            loss_factor *=
                (2.0 / PI) * acos(exp(nblades * (-node.distance_from_hub) / (2.0 * hub_radius * fabs(sin_phi))));
        }

        // update induction factors

        double tol_induction = 1e-6;  // tolerance for induction variables to avoid singularities
        // axial induction, based on AeroDyn v15 implementation
        double kk = node.chord_solidity * cn / (4.0 * loss_factor * pow(sin_phi, 2));
        if (kk <= 2.0 / 3.0) {
            if (fabs(kk + 1.0) < tol_induction) {
                aa = copysign(aa_max, -(1.0 + kk));
            } else {
                aa = kk / (1.0 + kk);
            }
            if (kk < -1.0) {
                throw std::runtime_error("Not valid BEMT solution for axial induction.");
            }
        } else {
            double ff = loss_factor;
            double temp = 2.0 * ff * kk;
            double g1 = temp - (10.0 / 9.0 - ff);
            double g2 = temp - (4.0 / 3.0 - ff) * ff;
            double g3 = temp - (25.0 / 9.0 - 2.0 * ff);

            if (fabs(g3) < tol_induction) {
                aa = 1.0 - 0.5 / sqrt(g2);
            } else {
                aa = (g1 - sqrt(fabs(g2))) / g3;
            }
        }

        // @todo fix tangential induction factor calculation (convergence)
        //// tangential induction
        // if (fabs(cos_phi) < tol_induction) {
        //     ap = -1.0;
        // } else {
        //     double kp = element.chord_solidity * ct / (4.0 * loss_factor * sin_phi * cos_phi);
        //     if (local_velocity_rotor.y() < 0.0) {
        //         kp = -kp;
        //     }
        //     if (fabs(kp - 1.0) < tol_induction) {
        //         ap = copysign(ap_max, 1.0 - kp);
        //     } else {
        //         ap = kp / (1.0 - kp);
        //     }
        // }
        ap = 0.0;  // deactivate tangential induction

        // apply limits on induction factors
        if (aa > aa_max) {
            aa = aa_max;
        } else if (aa < aa_min) {
            aa = aa_min;
        }
        if (ap < ap_min) {
            ap = ap_min;
        } else if (ap > ap_max) {
            ap = ap_max;
        }

        if ((fabs(alpha - alpha_previous) <= tol_rel * fabs(std::max(alpha_previous, alpha))) ||
            (fabs(alpha - alpha_previous) <= tol_abs)) {
            break;
        } else if (ii >= max_iter) {
            std::cout << "Warning: could not converge to new induction factor after " + std::to_string(ii) +
                             " iterations. Axial: " + std::to_string(aa) +
                             ", previous: " + std::to_string(aa_previous) + ". Tangential: " + std::to_string(ap) +
                             ", previous: " + std::to_string(ap_previous) + ". Alpha: " + std::to_string(alpha) +
                             ", previous: " + std::to_string(alpha_previous) + ". Local velocity in: (" +
                             std::to_string(local_velocity_rotor0.x()) + ", " +
                             std::to_string(local_velocity_rotor0.y()) + ")."
                      << std::endl;
        }
    }

    // store induction factors for starting point of next time iteration
    node.induction_factor_axial = aa;
    node.induction_factor_tangential = ap;

    // return velocity in local
    return local_velocity_rotor;
}

void seahowl::aero::apply_tower_shadow_effect_on_wind(Vector3d& wind_velocity,
                                                      const Vector3d& position,
                                                      double blade_azimuth,
                                                      const seahowl::aero::TowerAero& tower_aero) {
    if (blade_azimuth > PI / 2.0 || blade_azimuth < -PI / 2.0) {
        // get wind velocity in tower reference frame
        auto& towertop_position = tower_aero.elements.back().properties.coordinates;
        auto& towertop_rotation = tower_aero.elements.back().properties.rotation;
        auto wind_velocity_tower0 = towertop_rotation.inverse() * wind_velocity;
        // only take wind velocity perpendicular to tower axis
        auto wind_velocity_tower = wind_velocity_tower0;
        wind_velocity_tower.Set(0.0, wind_velocity.y(), wind_velocity_tower.z());

        // project element coordinates to tower reference frame
        auto& tower_top = tower_aero.elements.back();
        auto coordinates_projected = -(towertop_rotation.inverse() * (position - towertop_position));

        // find tower radius
        auto tower_length =
            (tower_aero.reference_points.back().coordinates - tower_aero.reference_points.front().coordinates).Length();
        if (coordinates_projected.x() > 0.0) {
            std::vector<double> fractions{1.0 - (tower_length - coordinates_projected.x()) / tower_length};
            auto tower_radius =
                seahowl::core::get_discretized_points(fractions, tower_aero.reference_points)[0].diameter / 2.0;
            auto xx = coordinates_projected.z();
            auto xx2 = pow(xx, 2);
            auto yy = coordinates_projected.y();
            auto yy2 = pow(yy, 2);
            wind_velocity_tower =
                (wind_velocity_tower + wind_velocity_tower * pow(tower_radius, 2) / pow(yy2 + xx2, 2) *
                                           Vector3d(0.0, (-2.0 * xx * yy), (yy2 - xx2)));

            // correct wind velocity
            wind_velocity = towertop_rotation * wind_velocity_tower;
        }
    }
}
