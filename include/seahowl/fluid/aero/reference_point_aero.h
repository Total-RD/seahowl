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
#include "seahowl/fluid/aero/airfoil.h"
#include "seahowl/fluid/hydro/morison.h"

// forward declarations
namespace seahowl {
namespace fluid {
namespace aero {
struct AirfoilProperties;
}  // namespace aero
}  // namespace fluid
namespace aero = fluid::aero;
}  // namespace seahowl

namespace seahowl {
namespace fluid {
namespace aero {

/**
 * @brief Blade aero reference point.
 *
 * The coordinate system used here is the IEC reference coordinate system for wind turbines:
 * x-axis: flapwise pointing towards nacelle,
 * y-axis: edgewise pointing towards trailing edge,
 * z-axis: longitudinal pointing towards blade tip.
 * (0, 0, 0) is at the root of the blade.
 */
struct BladeReferencePointAero {
    /** @brief Fraction (normalized abscissa along longitudinal axis of component) of reference point. */
    double fraction = 0.0;
    /** @brief Coordinates of reference point [m] */
    Vector3d coordinates{0.0, 0.0, 0.0};
    /** @brief Offset (x, y) for the aerodynamic center of blade at reference point [m] */
    Vector2d offset_aero{0.0, 0.0};
    /** @brief Chord of blade at reference point [m] */
    double chord = 0.0;
    /** @brief Structural twist angle of blade at reference point [rad] */
    double structural_twist = 0.0;
    /** @brief Airfoil properties of blade at reference point. */
    std::vector<AirfoilProperties> airfoil_properties{};

    /**
     * @brief Constructor.
     */
    BladeReferencePointAero();

    BladeReferencePointAero operator*(const double factor) const;
    BladeReferencePointAero operator+(const BladeReferencePointAero& other) const;
};

/** @brief Tower aerodynamic element */
struct TowerReferencePointAero {
    /** @brief Fraction (normalized abscissa along longitudinal axis of component) of reference point. */
    double fraction = 0.0;
    /** @brief Coordinates of reference point [m] */
    Vector3d coordinates{0.0, 0.0, 0.0};
    /** @brief Velocity of reference point [m/s] */
    Vector3d velocity{0.0, 0.0, 0.0};
    /** @brief Rotation of reference point. */
    Quaternion rotation{0.0, 0.0, 0.0, 0.0};
    /** @brief Diameter of tower at reference point [m] */
    double diameter = 0.0;
    /** @brief Coefficients. */
    hydro::HydroCoefficients coefficients;

    /**
     * @brief Constructor.
     */
    TowerReferencePointAero();

    TowerReferencePointAero operator*(const double factor) const;
    TowerReferencePointAero operator+(const TowerReferencePointAero& other) const;
};

}  // namespace aero
}  // namespace fluid
}  // namespace seahowl
