#include "seahowl/aero/airfoil.h"

#include <stdexcept>
#include <string>

using seahowl::aero::AirfoilCoefficients;
using seahowl::aero::AirfoilProperties;

AirfoilCoefficients::AirfoilCoefficients() {}

AirfoilCoefficients::~AirfoilCoefficients() {}

AirfoilCoefficients AirfoilCoefficients::operator*(const double factor) const {
    AirfoilCoefficients new_point = *this;
    // new_point.alpha *= factor;
    new_point.lift *= factor;
    new_point.drag *= factor;
    new_point.added_mass *= factor;
    return new_point;
}

AirfoilCoefficients AirfoilCoefficients::operator+(const AirfoilCoefficients& other) const {
    AirfoilCoefficients new_point = *this;
    /* new_point.reynolds_number += other.reynolds_number; */
    // new_point.alpha += other.alpha;
    new_point.lift += other.lift;
    new_point.drag += other.drag;
    new_point.added_mass += other.added_mass;
    return new_point;
};

AirfoilProperties::AirfoilProperties() {}

AirfoilProperties::~AirfoilProperties() {}

AirfoilProperties AirfoilProperties::operator*(const double factor) const {
    AirfoilProperties new_point = *this;
    /* new_point.reynolds_number *= factor; */
    for (int ii = 0; ii < coefficients_list.size(); ii++) {
        new_point.coefficients_list[ii] = coefficients_list[ii] * factor;
    }
    return new_point;
};
AirfoilProperties AirfoilProperties::operator+(const AirfoilProperties& other) const {
    AirfoilProperties new_point = *this;
    /* new_point.reynolds_number += other.reynolds_number; */
    int idx1 = 0;
    int idx2 = 0;
    new_point.coefficients_list.clear();
    while (idx1 + idx2 < this->coefficients_list.size() - 1 + other.coefficients_list.size() - 1) {
        auto& coeffs1 = this->coefficients_list[idx1];
        auto& coeffs2 = other.coefficients_list[idx2];
        if (abs(coeffs1.alpha - coeffs2.alpha) < 1e-6) {
            new_point.coefficients_list.push_back(coeffs1 + coeffs2);
            if (idx1 + 1 < this->coefficients_list.size() && idx2 + 1 < other.coefficients_list.size()) {
                if (this->coefficients_list[idx1 + 1].alpha > other.coefficients_list[idx2 + 1].alpha) {
                    idx2 += 1;
                } else if (this->coefficients_list[idx1 + 1].alpha < other.coefficients_list[idx2 + 1].alpha) {
                    idx1 += 1;
                } else {
                    idx1 += 1;
                    idx2 += 1;
                }
            }
        } else {
            if (coeffs1.alpha < coeffs2.alpha) {
                auto& coeffs2_previous = other.coefficients_list[idx2 - 1];
                double alpha_range = coeffs2.alpha - coeffs2_previous.alpha;
                double w1 = (coeffs1.alpha - coeffs2_previous.alpha) / alpha_range;
                double w2 = (coeffs2.alpha - coeffs1.alpha) / alpha_range;
                auto coeffs = coeffs1 + (coeffs2_previous * w1 + coeffs2 * w2);
                new_point.coefficients_list.push_back(coeffs);
                idx1 += 1;
            } else if (coeffs2.alpha < coeffs1.alpha) {
                auto& coeffs1_previous = this->coefficients_list[idx1 - 1];
                double alpha_range = coeffs1.alpha - coeffs1_previous.alpha;
                double w1 = (coeffs2.alpha - coeffs1_previous.alpha) / alpha_range;
                double w2 = (coeffs1.alpha - coeffs2.alpha) / alpha_range;
                auto coeffs = coeffs2 + (coeffs1_previous * w1 + coeffs1 * w2);
                new_point.coefficients_list.push_back(coeffs);
                idx2 += 1;
            }
        }
    }
    return new_point;
};

AirfoilCoefficients AirfoilProperties::find_coefficients(double alpha) {
    for (auto ii = 0; ii < coefficients_list.size() - 1; ii++) {
        double alpha1 = coefficients_list[ii].alpha;
        double alpha2 = coefficients_list[ii + 1].alpha;
        if (alpha1 <= alpha && alpha <= alpha2) {
            double alpha_range = alpha2 - alpha1;
            double weight1 = 1.0 - (alpha - alpha1) / alpha_range;
            double weight2 = 1.0 - (alpha2 - alpha) / alpha_range;
            AirfoilCoefficients coefficients;
            coefficients.alpha = coefficients_list[ii].alpha * weight1 + coefficients_list[ii + 1].alpha * weight2;
            coefficients.lift = coefficients_list[ii].lift * weight1 + coefficients_list[ii + 1].lift * weight2;
            coefficients.drag = coefficients_list[ii].drag * weight1 + coefficients_list[ii + 1].drag * weight2;
            coefficients.added_mass =
                coefficients_list[ii].added_mass * weight1 + coefficients_list[ii + 1].added_mass * weight2;
            return coefficients;
        }
    }
    throw std::runtime_error("Could not find alpha value (" + std::to_string(alpha) + ") for airfoil.");
}
