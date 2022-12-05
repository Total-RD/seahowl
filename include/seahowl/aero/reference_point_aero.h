#pragma once

#include <seahowl/core/reference_point.h>

namespace seahowl {
namespace aero {

/**@brief Reference aerodynamic (DOF) point for blade */
struct BladeReferencePointAero {
    double fraction;                                    ///< Fraction (normalized abscissa)
    chrono::ChVector<double> coordinates;               ///< Point coordinates
    chrono::ChQuaternion<double> rotation;              ///< Rotation
    double chord;                                       ///< Chord length
    double structural_twist;                            ///< Twist
    std::vector<AirfoilProperties> airfoil_properties;  ///< Airfoil properties for each elements

    BladeReferencePointAero();
    BladeReferencePointAero(seahowl::core::BladeReferencePoint& point);
    ~BladeReferencePointAero();

    BladeReferencePointAero operator*(const double factor) const;
    BladeReferencePointAero operator+(const BladeReferencePointAero& other) const;
};

/**@brief Tower aerodynamic element */
struct TowerReferencePointAero {
    double fraction;                        ///< Fraction (normalized abscissa)
    chrono::ChVector<double> coordinates;   ///< Reference point coordinates
    chrono::ChQuaternion<double> rotation;  ///< Rotation of reference point
    chrono::ChVector<double> velocity;      ///< Velocity of reference point
    double diameter = 0.0;                  ///< Diameter of tower at reference point
    double drag_coefficient = 0.0;          ///< Drag coefficient of tower at reference point

    TowerReferencePointAero();
    TowerReferencePointAero(seahowl::core::TowerReferencePoint& point);
    ~TowerReferencePointAero();

    TowerReferencePointAero operator*(const double factor) const;
    TowerReferencePointAero operator+(const TowerReferencePointAero& other) const;
};

}  // namespace aero
}  // namespace seahowl
