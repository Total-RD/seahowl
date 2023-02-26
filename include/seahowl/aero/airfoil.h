#pragma once

#include <vector>

namespace seahowl {
namespace aero {

/**
 * @brief AirFoil coefficients.
 *
 * Holds airfoil properties for a given angle of attack.
 */
struct AirfoilCoefficients {
    /** @brief Angle of attack. */
    double alpha = 0.0;
    /** @brief Lift coefficient (Cl). */
    double lift = 0.0;
    /** @brief Drag coefficient (Cd). */
    double drag = 0.0;
    /** @brief Added mass coefficient. */
    double added_mass = 0.0;

    /**
     * @brief Constructor.
     */
    AirfoilCoefficients();

    AirfoilCoefficients operator*(const double factor) const;
    AirfoilCoefficients operator+(const AirfoilCoefficients& other) const;
};

/**
 * @brief AirFoil properties.
 *
 * Holds airfoil properties for a tabulated list of angle of attack.
 */
struct AirfoilProperties {
    /** @brief Reynolds number. */
    double reynolds_number = 0.0;
    /** @brief Tabulated list of airfoil coefficients (each with a different angle of attack). */
    std::vector<AirfoilCoefficients> coefficients_list{};

    /**
     * @brief Constructor.
     */
    AirfoilProperties();

    /**
     * @brief Returns airfoil coefficients for a given angle of attack.
     *
     * @param[in] alpha Angle of attack.
     */
    AirfoilCoefficients find_coefficients(double alpha);

    AirfoilProperties operator*(const double factor) const;
    AirfoilProperties operator+(const AirfoilProperties& other) const;
};

}  // namespace aero
}  // namespace seahowl
