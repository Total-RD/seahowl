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
    /** @brief Moment coefficient. */
    double moment = 0.0;

    /**
     * @brief Constructor.
     */
    AirfoilCoefficients();

    /**
     * @brief Multiplies all coefficients by a scalar factor.
     *
     * @param[in] factor Scalar multiplication factor.
     * @return New AirfoilCoefficients with scaled values.
     */
    AirfoilCoefficients operator*(const double factor) const;

    /**
     * @brief Adds two AirfoilCoefficients element-wise.
     *
     * @param[in] other AirfoilCoefficients to add.
     * @return New AirfoilCoefficients with summed values.
     */
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

    /**
     * @brief Multiplies all properties by a scalar factor.
     *
     * @param[in] factor Scalar multiplication factor.
     * @return New AirfoilProperties with scaled values.
     */
    AirfoilProperties operator*(const double factor) const;

    /**
     * @brief Adds two AirfoilProperties element-wise (for interpolation).
     *
     * @param[in] other AirfoilProperties to add.
     * @return New AirfoilProperties with summed values.
     */
    AirfoilProperties operator+(const AirfoilProperties& other) const;
};

}  // namespace aero
}  // namespace seahowl
