#pragma once

#include "seahowl/commons/numerics.h"

namespace seahowl {
namespace elasto {

/**
 * @brief Elasto reference point base class.
 *
 * All elasto reference point classes are derived from this class.
 */
struct ReferencePointElasto {
    /** @brief Coordinates of reference point. */
    Vector3d coordinates{0.0, 0.0, 0.0};
    /** @brief Fraction (normalized abscissa along longitudinal axis of component) of reference point. */
    double fraction = 0.0;

    /**
     * @brief Constructor.
     */
    ReferencePointElasto();

    ReferencePointElasto operator*(const double factor) const;
    ReferencePointElasto operator+(const ReferencePointElasto& other) const;
};

/**
 * @brief Blade elasto reference point.
 *
 * The coordinate system used here is the IEC reference coordinate system for wind turbines:
 * x-axis: flapwise pointing towards nacelle,
 * y-axis: edgewise pointing towards trailing edge,
 * z-axis: longitudinal pointing towards blade tip.
 * (0, 0, 0) is at the root of the blade.
 */
struct BladeReferencePointElasto : ReferencePointElasto {
    /** @brief Offset (x, y) for the center of elasticity of blade at reference point. */
    Vector2d offset_elastic{0.0, 0.0};
    /** @brief Offset (x, y) for the center of gravity of blade at reference point. */
    Vector2d offset_gravity{0.0, 0.0};
    /** @brief Stiffness matrix of blade at reference point. */
    Eigen::Matrix<double, 6, 6> stiffness_matrix = Eigen::Matrix<double, 6, 6>::Zero();
    /** @brief Mass matrix of blade at reference point. */
    Eigen::Matrix<double, 6, 6> mass_matrix = Eigen::Matrix<double, 6, 6>::Zero();
    /** @brief Structural twist angle of blade at reference point. */
    double structural_twist = 0.0;
    /** @brief Flapwise (bending and shear) stiffness-proportial damping coefficients at reference point.*/
    double damping_flapwise = 0.005;
    /** @brief Edgewise (bending and shear) stiffness-proportial damping coefficients at reference point.*/
    double damping_edgewise = 0.005;
    /** @brief Axial stiffness-proportial damping coefficients at reference point.*/
    double damping_axial = 0.005;
    /** @brief Torsional stiffness-proportial damping coefficients at reference point.*/
    double damping_torsion = 0.005;
    /** @brief Mass-proportial damping coefficients at reference point.*/
    double damping_mass = 0.0;

    /**
     * @brief Constructor.
     */
    BladeReferencePointElasto();

    BladeReferencePointElasto operator*(const double factor) const;
    BladeReferencePointElasto operator+(const BladeReferencePointElasto& other) const;
};

/**
 * @brief Tower elasto reference point.
 */
struct TowerReferencePointElasto : ReferencePointElasto {
    /** @brief Lineic density of tower at reference point. */
    double density = 0.0;
    /** @brief Axial stiffness of tower at reference point. */
    double stiffness_axial = 0.0;
    /** @brief Fore-aft stiffness of tower at reference point. */
    double stiffness_foreaft = 0.0;
    /** @brief Side-side stiffness of tower at reference point. */
    double stiffness_sideside = 0.0;
    /** @brief Torsional stiffness of tower at reference point. */
    double stiffness_torsion = 0.0;
    /** @brief Fore-aft shear stiffness of tower at reference point. */
    double stiffness_foreaft_shear = 0.0;
    /** @brief Side-side shear stiffness of tower at reference point. */
    double stiffness_sideside_shear = 0.0;
    /** @brief Fore-aft inertia of tower at reference point. */
    double inertia_foreaft = 0.0;
    /** @brief Side-side inertia of tower at reference point. */
    double inertia_sideside = 0.0;
    /** @brief Fore-aft (bending and shear) stiffness-proportial damping coefficients at reference point.*/
    double damping_foreaft = 0.005;
    /** @brief Side-side (bending and shear) stiffness-proportial damping coefficients at reference point.*/
    double damping_sideside = 0.005;
    /** @brief Axial stiffness-proportial damping coefficients at reference point.*/
    double damping_axial = 0.005;
    /** @brief Torsional stiffness-proportial damping coefficients at reference point.*/
    double damping_torsion = 0.005;
    /** @brief Mass-proportial damping coefficients at reference point.*/
    double damping_mass = 0.0;

    /**
     * @brief Constructor.
     */
    TowerReferencePointElasto();

    TowerReferencePointElasto operator*(const double factor) const;
    TowerReferencePointElasto operator+(const TowerReferencePointElasto& other) const;

    /**
     * @brief Set properties for hollow cylinder.
     *
     * @param[in] density Density of material (kg/m3).
     * @param[in] young_modulus Young's modulus of material (Pa).
     * @param[in] poisson_ratio Poisson ratio of material (-).
     * @param[in] outer_diameter Outer diameter of cylinder (m).
     * @param[in] thickness Thickness of cylinder (m).
     * @param[in] shear Whether to include shear or not.
     */
    void set_properties_cylinder(double density,
                                 double young_modulus,
                                 double poisson_ratio,
                                 double outer_diameter,
                                 double thickness,
                                 bool shear = false);
};

}  // namespace elasto
}  // namespace seahowl
