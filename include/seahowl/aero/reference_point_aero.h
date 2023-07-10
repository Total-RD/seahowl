#pragma once

#include "seahowl/commons/numerics.h"
#include "seahowl/aero/airfoil.h"

// forward declarations
namespace seahowl {
namespace aero {
struct AirfoilProperties;
}  // namespace aero
}  // namespace seahowl

namespace seahowl {
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
    /** @brief Coordinates of reference point. */
    Vector3d coordinates{0.0, 0.0, 0.0};
    /** @brief Offset (x, y) for the aerodynamic center of blade at reference point. */
    Vector2d offset_aero{0.0, 0.0};
    /** @brief Chord of blade at reference point. */
    double chord = 0.0;
    /** @brief Structural twist angle of blade at reference point. */
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

/**@brief Tower aerodynamic element */
struct TowerReferencePointAero {
    /** @brief Fraction (normalized abscissa along longitudinal axis of component) of reference point. */
    double fraction = 0.0;
    /** @brief Coordinates of reference point. */
    Vector3d coordinates{0.0, 0.0, 0.0};
    /** @brief Velocity of reference point. */
    Vector3d velocity{0.0, 0.0, 0.0};
    /** @brief Rotation of reference point. */
    Quaternion rotation{0.0, 0.0, 0.0, 0.0};
    /** @brief Diameter of tower at reference point. */
    double diameter = 0.0;
    /** @brief Drag coefficient of tower at reference point. */
    double drag_coefficient = 0.0;

    /**
     * @brief Constructor.
     */
    TowerReferencePointAero();

    TowerReferencePointAero operator*(const double factor) const;
    TowerReferencePointAero operator+(const TowerReferencePointAero& other) const;
};

}  // namespace aero
}  // namespace seahowl
