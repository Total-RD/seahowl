#pragma once

#include <chrono/fea/ChElementBeamTaperedTimoshenko.h>
#include <seahowl/aero/airfoil.h>  ///< @todo Bad dependencyof aero on core ?

#include <vector>

namespace seahowl {
namespace core {

/**@brief Reference point (mesh) for Blade */
struct BladeReferencePoint {
    chrono::ChVector<double> coordinates{0.0, 0.0, 0.0};               ///< Reference coordinates
    chrono::ChVector2<double> offset_elastic{0.0, 0.0};                ///< Offset of center of elasticity
    chrono::ChVector2<double> offset_gravity{0.0, 0.0};                ///< Offset of center of gravity
    chrono::ChVector2<double> offset_aero{0.0, 0.0};                   ///< Aerodynamic offset
    chrono::ChMatrixNM<double, 6, 6> stiffness_matrix;                 ///< Stiffness matrix
    chrono::ChMatrixNM<double, 6, 6> mass_matrix;                      ///< Mass matrix
    double fraction = 0.0;                                             ///< Fraction (normalized abscissa)
    double structural_twist = 0.0;                                     ///< Twist
    double chord = 0.0;                                                ///< Chord
    chrono::fea::DampingCoefficients damping_coefficients;             ///< Damping coefficients
    std::vector<seahowl::aero::AirfoilProperties> airfoil_properties;  ///< Airfoil properties for each element

    BladeReferencePoint();
    ~BladeReferencePoint();

    BladeReferencePoint operator*(const double factor) const;
    BladeReferencePoint operator+(const BladeReferencePoint& other) const;
};

/**@brief Reference point (mesh) for Tower */
struct TowerReferencePoint {
    chrono::ChVector<double> coordinates;  ///< Coordinates of reference point
    double fraction = 0.0;                 ///< Fraction (normalized abscissa along tower)
    // elasto
    double density = 0.0;                                   ///< Density
    double stiffness_axial = 0.0;                           ///< Axial stiffness
    double stiffness_foreaft = 0.0;                         ///< Fore-aft stiffness
    double stiffness_sideside = 0.0;                        ///< Side-side stiffness
    double stiffness_torsion = 0.0;                         ///< Torsional stiffness
    chrono::fea::DampingCoefficients damping_coefficients;  ///< Damping coefficients
    // aero
    double diameter = 0.0;
    double drag_coefficient = 0.0;

    TowerReferencePoint();
    ~TowerReferencePoint();

    TowerReferencePoint operator*(const double factor) const;
    TowerReferencePoint operator+(const TowerReferencePoint& other) const;
};

}  // namespace core
}  // namespace seahowl
