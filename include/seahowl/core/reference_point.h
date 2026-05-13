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
#include "seahowl/commons/numerics.h"

// Standard library
#include <vector>

// forward declarations
namespace seahowl {
namespace fluid {
namespace aero {
struct AirfoilProperties;
}  // namespace aero
}  // namespace fluid
}  // namespace seahowl

namespace seahowl {
namespace core {

/**
 * @brief Blade reference point.
 *
 * The coordinate system used here is the IEC reference coordinate system for wind turbines:
 * x-axis: flapwise pointing towards nacelle,
 * y-axis: edgewise pointing towards trailing edge,
 * z-axis: longitudinal pointing towards blade tip.
 * (0, 0, 0) is at the root of the blade.
 */
struct BladeReferencePoint {
    // elasto
    //
    /** @brief Coordinates of reference point [m] */
    Vector3d coordinates{0.0, 0.0, 0.0};
    /** @brief Offset (x, y) for the center of elasticity of blade at reference point [m] */
    Vector2d offset_elastic{0.0, 0.0};
    /** @brief Offset (x, y) for the center of gravity of blade at reference point [m] */
    Vector2d offset_gravity{0.0, 0.0};
    /** @brief Offset (x, y) for the aerodynamic center of blade at reference point [m] */
    Vector2d offset_aero{0.0, 0.0};
    /** @brief Stiffness matrix of blade at reference point. */
    Eigen::Matrix<double, 6, 6> stiffness_matrix;
    /** @brief Mass matrix of blade at reference point. */
    Eigen::Matrix<double, 6, 6> mass_matrix;
    /** @brief Fraction (normalized abscissa along longitudinal axis of component) of reference point. */
    double fraction = 0.0;
    /** @brief Structural twist angle of blade at reference point [rad] */
    double structural_twist = 0.0;

    // aero
    //
    /** @brief Chord of blade at reference point [m] */
    double chord = 0.0;
    /** @brief Damping coefficients at reference point {along x, along y, along z, about x, mass-proportional}.*/
    std::vector<double> damping_coefficients{0.03, 0.03, 0.03, 0.06, 0.0};
    /** @brief Airfoil properties of blade at reference point. */
    std::vector<seahowl::fluid::aero::AirfoilProperties> airfoil_properties{};

    /**
     * @brief Constructor.
     */
    BladeReferencePoint();

    BladeReferencePoint operator*(const double factor) const;
    BladeReferencePoint operator+(const BladeReferencePoint& other) const;
};

/**
 * @brief Tower reference point.
 */
struct TowerReferencePoint {
    // elasto
    //
    /** @brief Coordinates of reference point [m] */
    Vector3d coordinates{0.0, 0.0, 0.0};
    /** @brief Fraction (normalized abscissa along longitudinal axis of component) of reference point. */
    double fraction = 0.0;
    /** @brief Lineic density of tower at reference point [kg/m] */
    double density = 0.0;
    /** @brief Axial stiffness of tower at reference point. */
    double stiffness_axial = 0.0;
    /** @brief Fore-aft stiffness of tower at reference point. */
    double stiffness_foreaft = 0.0;
    /** @brief Side-side stiffness of tower at reference point. */
    double stiffness_sideside = 0.0;
    /** @brief Torsional stiffness of tower at reference point. */
    double stiffness_torsion = 0.0;
    /** @brief Damping coefficients at reference point {along x, along y, along z, about x, mass-proportional}.*/
    std::vector<double> damping_coefficients{0.03, 0.03, 0.03, 0.06, 0.0};

    // aero
    //
    /** @brief Diameter of tower at reference point [m] */
    double diameter = 0.0;
    /** @brief Drag coefficient of tower at reference point. */
    double drag_coefficient = 0.0;

    /**
     * @brief Constructor.
     */
    TowerReferencePoint();

    TowerReferencePoint operator*(const double factor) const;
    TowerReferencePoint operator+(const TowerReferencePoint& other) const;
};

}  // namespace core
}  // namespace seahowl
