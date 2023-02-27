#pragma once

#include <seahowl/aero/airfoil.h>
#include <seahowl/aero/blade_aero.h>
#include <seahowl/aero/tower_aero.h>
#include <seahowl/utils.h>

namespace seahowl {
namespace aero {

/**
 * @brief Returns phi (angle of fluid velocity from zero-pitch axis).
 *
 * @param[in] fluid_velocity Velocity of fluid.
 */
double get_phi(const Vector2d& fluid_velocity);

/**
 * @brief Returns angle of attack from phi and blade pitch.
 *
 * @param[in] phi Angle of fluid velocity from zero-pitch axis.
 * @param[in] picth Picth of blade.
 */
double get_alpha_from_phi(const double phi, const double pitch);

/**
 * @brief Returns angle of attack from fluid velocity and blade pitch.
 *
 * @param[in] fluid_velocity Velocity of fluid.
 * @param[in] picth Picth of blade.
 */
double get_alpha(const Vector2d& fluid_velocity, const double pitch);

/**
 * @brief Returns airfoil coefficients from angle of attack.
 *
 * @param[in] alpha Angle of attack.
 * @param[in] picth airfoil_properties Aero properties of airfoils.
 */
AirfoilCoefficients get_aero_coefficients_from_alpha(const double alpha,
                                                     std::vector<AirfoilProperties>& airfoil_properties);

/**
 * @brief Returns induced velocity.
 *
 * @param[out] node Node on which induced velocity in computed.
 * @param[in] local_velocity_rotor0 Local uninduced velocity at node.
 * @param[in] blade_pitch Pitch of blade on which node is placed.
 * @param[in] nblades Number of blade.
 * @param[in] tip_loss Whether to take tip loss into account or not.
 * @param[in] hub_loss Whether to take hub loss into account or not.
 */
Vector2d get_induced_velocity(BladeNodeAero& node,
                              const Vector2d& local_velocity_rotor0,
                              const double blade_pitch = 0.0,
                              const size_t nblades = 3,
                              const bool tip_loss = true,
                              const bool hub_loss = true);

/**
 * @brief Apply tower shadow on wind velocity.
 *
 * @param[out] wind_velocity Wind velocity on which to apply tower shadow.
 * @param[in] position Position of at which wind velocity was extracted.
 * @param[in] blade_azimuth Azimuth of blade.
 * @param[in] tower_aero Tower from which tower shadow effect is felt.
 */
void apply_tower_shadow_effect_on_wind(Vector3d& wind_velocity,
                                       const Vector3d& position,
                                       double blade_azimuth,
                                       const TowerAero& tower_aero);
}  // namespace aero
}  // namespace seahowl
