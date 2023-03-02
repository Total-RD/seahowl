#pragma once

#include <seahowl/core/reference_point.h>

#include <seahowl/commons.h>

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
    Eigen::Matrix<double, 6, 6> stiffness_matrix;
    /** @brief Mass matrix of blade at reference point. */
    Eigen::Matrix<double, 6, 6> mass_matrix;
    /** @brief Structural twist angle of blade at reference point. */
    double structural_twist = 0.0;
    /** @brief Damping coefficients of blade at reference point. */
    std::vector<double> damping_coefficients{0.03, 0.03, 0.03, 0.06, 0.0};

    /**
     * @brief Constructor.
     */
    BladeReferencePointElasto();

    /**
     * @brief Constructor.
     *
     * @param[in] point General blade reference point holding elasto info.
     */
    BladeReferencePointElasto(const seahowl::core::BladeReferencePoint& point);

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
    /** @brief Damping coefficients of tower at reference point. */
    std::vector<double> damping_coefficients{0.03, 0.03, 0.03, 0.06, 0.0};

    /**
     * @brief Constructor.
     */
    TowerReferencePointElasto();

    /**
     * @brief Constructor.
     *
     * @param[in] point General tower reference point holding elasto info.
     */
    TowerReferencePointElasto(const seahowl::core::TowerReferencePoint& point);

    TowerReferencePointElasto operator*(const double factor) const;
    TowerReferencePointElasto operator+(const TowerReferencePointElasto& other) const;
};

}  // namespace elasto
}  // namespace seahowl
