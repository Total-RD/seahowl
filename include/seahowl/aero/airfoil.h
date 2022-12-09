#pragma once

#include <vector>

namespace seahowl {
namespace aero {

/**@brief AriFoil coefficients */
struct AirfoilCoefficients {
    double alpha = 0.0;       ///< Angle of attack
    double lift = 0.0;        ///< Lift coefficient Cl
    double drag = 0.0;        ///< Drag coefficient Cd
    double added_mass = 0.0;  ///< Added mass coefficient

    AirfoilCoefficients();
    ~AirfoilCoefficients();

    AirfoilCoefficients operator*(const double factor) const;
    AirfoilCoefficients operator+(const AirfoilCoefficients& other) const;
};

/**@brief AriFoil properties

@todo depends also of air density
*/
struct AirfoilProperties {
    double reynolds_number = 0.0;                          ///< Reynolds number
    std::vector<AirfoilCoefficients> coefficients_list{};  ///< Tabulated angle of attack (alpha) and corresponding
                                                           ///< lift, drag, added_mass coefficients

    AirfoilProperties();
    ~AirfoilProperties();

    AirfoilProperties operator*(const double factor) const;
    AirfoilProperties operator+(const AirfoilProperties& other) const;
    AirfoilCoefficients find_coefficients(double alpha);
};

}  // namespace aero
}  // namespace seahowl
