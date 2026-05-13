// SPDX-License-Identifier: Apache-2.0
// Copyright 2022–2026 TotalEnergies
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//

#pragma once

// SEAHOWL headers
#include "seahowl/fluid/aero/blade_aero.h"
#include "seahowl/fluid/aero/tower_aero.h"
#include "seahowl/commons/numerics.h"

// forward declarations
namespace seahowl {
namespace fluid {
namespace aero {
struct AirfoilProperties;
struct AirfoilCoefficients;
}  // namespace aero
}  // namespace fluid
namespace aero = fluid::aero;
}  // namespace seahowl

namespace seahowl {
namespace fluid {
namespace aero {

/**
 * @brief Returns phi (angle of fluid velocity from zero-pitch axis).
 *
 * @param[in] fluid_velocity Velocity of fluid in 2D airfoil coordinates.
 * @return Angle phi [rad]
 */
double get_phi(const Vector2d& fluid_velocity);

/**
 * @brief Returns angle of attack from phi and blade pitch.
 *
 * @param[in] phi Angle of fluid velocity from zero-pitch axis [rad]
 * @param[in] pitch Pitch angle of blade [rad]
 * @return Angle of attack [rad]
 */
double get_alpha_from_phi(const double phi, const double pitch);

/**
 * @brief Returns angle of attack from fluid velocity and blade pitch.
 *
 * @param[in] fluid_velocity Velocity of fluid in 2D airfoil plane coordinates.
 * @param[in] pitch Pitch angle of blade [rad]
 * @return Angle of attack [rad]
 */
double get_alpha(const Vector2d& fluid_velocity, const double pitch);

/**
 * @brief Returns airfoil coefficients from angle of attack.
 *
 * Interpolates airfoil coefficients (Cl, Cd, Cm) from tabulated data
 * based on the given angle of attack.
 *
 * @param[in] alpha Angle of attack [rad]
 * @param[in] airfoil_properties Vector of airfoil properties containing coefficient tables.
 * @return Interpolated airfoil coefficients at the given angle of attack.
 */
AirfoilCoefficients get_aero_coefficients_from_alpha(const double alpha,
                                                     std::vector<AirfoilProperties>& airfoil_properties);

/**
 * @brief Computes induced velocity using BEMT (Blade Element Momentum Theory).
 *
 * Iteratively computes the induced velocity at a blade node using momentum theory
 * with optional tip and hub loss corrections. Updates induction factors on the node.
 *
 * @param[in,out] node Blade aerodynamic node; induction factors are updated.
 * @param[in] local_velocity_rotor0 Local uninduced velocity at node in 2D rotor plane coordinates.
 * @param[in] pitch Pitch angle of blade [rad]
 * @param[in] nblades Number of blades on rotor.
 * @param[in] tip_loss Whether to apply Prandtl tip loss correction.
 * @param[in] hub_loss Whether to apply Prandtl hub loss correction.
 * @return Induced velocity in 2D rotor plane coordinates (axial, tangential).
 */
Vector2d get_induced_velocity(BladeNodeAero& node,
                              const Vector2d& local_velocity_rotor0,
                              const double pitch = 0.0,
                              const size_t nblades = 3,
                              const bool tip_loss = true,
                              const bool hub_loss = true);

/**
 * @brief Applies tower shadow effect on wind velocity.
 *
 * Modifies the wind velocity to account for the aerodynamic shadow
 * cast by the tower on the downwind side.
 *
 * @param[in,out] wind_velocity Wind velocity vector to be modified by tower shadow effect.
 * @param[in] position Position at which wind velocity was extracted.
 * @param[in] tower_aero Tower aerodynamic model providing shadow geometry.
 */
void apply_tower_shadow_effect_on_wind(Vector3d& wind_velocity, const Vector3d& position, const TowerAero& tower_aero);
}  // namespace aero
}  // namespace fluid
}  // namespace seahowl
