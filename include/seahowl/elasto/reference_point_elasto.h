#pragma once

#include <seahowl/core/reference_point.h>

#include <chrono/fea/ChElementBeamTaperedTimoshenko.h>

namespace seahowl {
namespace elasto {

struct ReferencePointElasto {
    chrono::ChVector<double> coordinates{0.0, 0.0, 0.0};  ///< Coordinates of reference point
    double fraction = 0.0;                                ///< Fraction (normalized abscissa along component)

    ReferencePointElasto();
    ~ReferencePointElasto();

    ReferencePointElasto operator*(const double factor) const;
    ReferencePointElasto operator+(const ReferencePointElasto& other) const;
};

/**@brief Elastodynamic model DOF reference point */
struct BladeReferencePointElasto : ReferencePointElasto {
    chrono::ChVector2<double> offset_elastic{0.0, 0.0};     ///< Offset of center of elasticity
    chrono::ChVector2<double> offset_gravity{0.0, 0.0};     ///< Offset of center of gravity
    chrono::ChMatrixNM<double, 6, 6> stiffness_matrix;      ///< Stiffness matrix
    chrono::ChMatrixNM<double, 6, 6> mass_matrix;           ///< Mass matrix
    double structural_twist = 0.0;                          ///< Twist angle (radians)
    chrono::fea::DampingCoefficients damping_coefficients;  ///< Damping coefficients

    BladeReferencePointElasto();
    BladeReferencePointElasto(seahowl::core::BladeReferencePoint point);
    ~BladeReferencePointElasto();

    BladeReferencePointElasto operator*(const double factor) const;
    BladeReferencePointElasto operator+(const BladeReferencePointElasto& other) const;
};

/**@brief Tower (DOF) reference point */
struct TowerReferencePointElasto : ReferencePointElasto {
    double density = 0.0;                                   ///< Density
    double stiffness_axial = 0.0;                           ///< Axial stiffness
    double stiffness_foreaft = 0.0;                         ///< Fore-aft stiffness
    double stiffness_sideside = 0.0;                        ///< Side-side stiffness
    double stiffness_torsion = 0.0;                         ///< Torsional stiffness
    chrono::fea::DampingCoefficients damping_coefficients;  ///< Damping coefficients

    TowerReferencePointElasto();
    TowerReferencePointElasto(seahowl::core::TowerReferencePoint point);
    ~TowerReferencePointElasto();

    TowerReferencePointElasto operator*(const double factor) const;
    TowerReferencePointElasto operator+(const TowerReferencePointElasto& other) const;
};

}  // namespace elasto
}  // namespace seahowl