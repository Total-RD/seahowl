#include "seahowl/fluid/aero/bemt.h"

#include "seahowl/commons/utils.h"
#include "seahowl/fluid/aero/airfoil.h"
#include "seahowl/fluid/aero/blade_aero.h"
#include "seahowl/fluid/aero/tower_aero.h"

#include <spdlog/spdlog.h>

using seahowl::Vector2d;
using seahowl::Vector3d;
using seahowl::PI;

double seahowl::aero::get_phi(const Vector2d& fluid_velocity) {
    double phi = atan2(fluid_velocity.y(), fluid_velocity.x());
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
                                             const double pitch,
                                             const size_t nblades,
                                             const bool tip_loss,
                                             const bool hub_loss) {
    // local_velocity is in local element frame
    Vector2d local_velocity;
    Vector2d local_velocity_rotor;

    double tol_rel = 1e-3;
    double tol_abs = 1e-3;
    int max_iter = 100;
    double alpha = -1000.0;
    double alpha_previous;
    auto aa = node.induction_factor_axial;
    auto ap = node.induction_factor_tangential;
    // limits
    double aa_max = 1.0;
    double aa_min = 0.0;
    double ap_max = 1.0;
    double ap_min = -1.0;
    // reset induction factors if they were max or min
    if (aa >= aa_max || aa <= aa_min) {
        aa = 0.0;
    }
    if (ap >= ap_max || ap <= ap_min) {
        ap = 0.0;
    }
    for (int ii = 1; ii <= max_iter; ii++) {
        // store previous alpha
        alpha_previous = alpha;
        // store previous induction factors
        auto aa_previous = aa;
        auto ap_previous = ap;

        // local velocity updated with induction factors
        local_velocity_rotor = Vector2d(local_velocity_rotor0[0] * (1.0 + ap), local_velocity_rotor0[1] * (1.0 - aa));

        // get coefficients from angle of attack
        double phi = seahowl::aero::get_phi(local_velocity_rotor);
        alpha = seahowl::aero::get_alpha_from_phi(phi, pitch);
        auto coefficients = seahowl::aero::get_aero_coefficients_from_alpha(alpha, node.properties.airfoil_properties);

        // get drag and lift coefficients
        auto cl = coefficients.lift;
        auto cd = coefficients.drag;
        // projected to rotor local frame
        double cos_phi = cos(phi);
        double sin_phi = sin(phi);
        double cn = cl * cos_phi + cd * sin_phi;
        double ct = -cl * sin_phi + cd * cos_phi;

        double tol_induction = 1e-6;  // tolerance for induction variables to avoid singularities

        // losses
        double loss_factor = 1.0;
        // tip loss (with check that no division by zero will happen => y component of velocity > 0)
        if (tip_loss && fabs(sin_phi) > tol_induction) {
            // Prandtl's approximation for tip-loss factor=
            loss_factor *=
                (2.0 / PI) * acos(exp(nblades * (-node.distance_from_tip) / (2.0 * node.radius * fabs(sin_phi))));
        }
        // hub loss (with check that no division by zero will happen => y component of velocity > 0)
        if (hub_loss && fabs(sin_phi) > tol_induction) {
            // hub loss
            double hub_radius = (node.radius - node.distance_from_hub);
            loss_factor *=
                (2.0 / PI) * acos(exp(nblades * (-node.distance_from_hub) / (2.0 * hub_radius * fabs(sin_phi))));
        }

        // update induction factors

        // axial induction, based on and adapted from AeroDyn v15 implementation
        // first check if no division by zero (y component of velocity > 0)
        if (fabs(sin_phi) > tol_induction) {
            double kk = node.chord_solidity * cn / (4.0 * loss_factor * pow(sin_phi, 2));
            if (kk <= 2.0 / 3.0) {
                if (fabs(kk + 1.0) < tol_induction) {
                    aa = copysign(aa_max, -(1.0 + kk));
                } else {
                    aa = kk / (1.0 + kk);
                }
                if (kk < -1.0) {
                    // equivalent to aa > 1.0, not possible here => cap it
                    aa = aa_max;
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
        } else {
            aa = aa_max;
        }

        //// tangential induction
        if (fabs(cos_phi) < tol_induction) {
            ap = ap_min;
        } else if (fabs(sin_phi) < tol_induction) {
            ap = ap_max;
        } else {
            double kp = -node.chord_solidity * ct / (4.0 * loss_factor * sin_phi * cos_phi);
            if (local_velocity_rotor.y() < 0.0) {
                kp = -kp;
            }
            if (fabs(kp - 1.0) < tol_induction) {
                ap = copysign(ap_max, 1.0 - kp);
            } else {
                ap = kp / (1.0 - kp);
            }
        }

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
            spdlog::warn("Could not converge to new induction factor after {} iterations.", ii);
            spdlog::warn("    Local velocity: ({:.4}, {:.4}).", local_velocity_rotor0.x(), local_velocity_rotor0.y());
            spdlog::warn("    Alpha: {:.4} (previous: {:.4})", alpha, alpha_previous);
            spdlog::warn("    Axial: {:.4} (previous: {:.4})", aa, aa_previous);
            spdlog::warn("    Tangential: {:.4} (previous: {:.4})", ap, ap_previous);
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
                                                      const seahowl::aero::TowerAero& tower_aero) {
    // check that wind velocity is not ~zero
    double tol = 1e-6;
    if (wind_velocity.norm() < tol) {
        // do not affect wind velocity if below tolerance
        return;
    }

    // get wind velocity in tower reference frame
    auto& towertop = tower_aero.nodes.back();
    auto& towerbase = tower_aero.nodes.front();
    auto towertop_position = towertop.get_position();
    auto towerbase_position = towerbase.get_position();
    // tower axis assumption: tower is straight from towerbottom to towertop)
    auto tower_axis = (towertop_position - towerbase_position).normalized();
    // get average tower velocity (for relative wind velocity)
    auto tower_velocity = Vector3d(0.0, 0.0, 0.0);
    for (auto& node : tower_aero.nodes) {
        tower_velocity += node.get_velocity();
    }
    tower_velocity /= (tower_aero.nodes.size());

    // only take wind velocity perpendicular to tower axis
    auto wind_velocity_tower =
        (wind_velocity - tower_velocity) - ((wind_velocity - tower_velocity).dot(tower_axis)) * (tower_axis);
    auto wind_axis_tower = wind_velocity_tower.normalized();
    auto wind_normal_tower = tower_axis.cross(wind_axis_tower);

    // get z positions along tower axis
    // rotate all positions around center of tower to get relative positions to tower center in IEC coordinate system
    auto z_position = (position - towerbase_position).dot(tower_axis);
    auto z_towertop = (towertop_position - towerbase_position).dot(tower_axis);
    if (z_position >= 0 && z_position <= z_towertop) {
        // get diameter at z position
        auto z_rel = z_position / z_towertop;
        std::vector<double> fractions{z_rel};
        auto tower_radius = seahowl::get_discretized_points(fractions, tower_aero.discretized_points)[0].diameter / 2.0;

        // find relative position
        auto position_projected = towerbase_position + z_rel * (towertop_position - towerbase_position);
        auto position_relative_projected = position - position_projected;
        // front position of point from tower
        auto xx = position_relative_projected.dot(wind_axis_tower);
        auto xx2 = pow(xx, 2);
        // side position of point from tower
        auto yy = position_relative_projected.dot(wind_normal_tower);
        auto yy2 = pow(yy, 2);

        // factor for tower shadow calculations
        auto factor = pow(tower_radius, 2) / pow(yy2 + xx2, 2);

        // wind shadow tangential to wind direction
        auto wind_shadow_x = factor * (yy2 - xx2) * wind_velocity_tower.norm();
        // wind shadow normal to wind direction
        auto wind_shadow_y = factor * (-2.0 * xx * yy) * wind_velocity_tower.norm();

        // total tower shadow contributions
        auto wind_shadow = wind_shadow_x * wind_axis_tower + wind_shadow_y * wind_normal_tower;

        // update wind velocity
        wind_velocity += wind_shadow;
    }
}
