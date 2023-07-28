#pragma once

#include <vector>
#include <stdexcept>
#include <algorithm>

namespace seahowl {

class Grid1D {
  public:
    Grid1D();
    Grid1D(std::vector<double> ticks);
    void set_ticks(std::vector<double> ticks);
    int get_index(double value);

  private:
    std::vector<double> ticks{};
};

/**
 * @brief Parametric discretization point.
 */
struct DiscretizationPoint {
    /** @brief Index of element. */
    int index = 0;
    /** @brief Abscissa along element within range [-1, +1], with -1 at node1, and +1 at node2. */
    double eta = 0;
};

std::vector<DiscretizationPoint> get_indice_and_positions(const std::vector<double>& discretization_fractions,
                                                          const std::vector<double>& reference_fractions);

template <typename T>
std::vector<T> get_discretized_points(const std::vector<double>& discretization_fractions,
                                      const std::vector<T>& reference_points) {
    if (discretization_fractions.size() == 0) {
        // discretize at centers of reference directly
        return reference_points;
    } else {
        std::vector<T> discretized_points;
        std::vector<double> reference_fractions;
        for (int ii = 0; ii < reference_points.size(); ii++) {
            reference_fractions.push_back(reference_points[ii].fraction);
        }
        // check bounds
        if (reference_fractions[0] != 0.0 || reference_fractions[reference_fractions.size() - 1] != 1.0) {
            throw std::runtime_error("Reference fractions must start with 0 and end with 1 but got " +
                                     std::to_string(reference_fractions[0]) + " and " +
                                     std::to_string(reference_fractions[reference_fractions.size() - 1]) + ".");
        }
        for (int ii = 0; ii < discretization_fractions.size(); ii++) {
            // find position of node
            // interpolate to node position from centers of reference at key fractions
            double fraction = discretization_fractions[ii];
            // check bounds
            if (fraction < 0.0 || fraction > 1.0) {
                throw std::runtime_error("Discretization fraction must be between 0 and 1 but was " +
                                         std::to_string(fraction) + ".");
            }
            auto idx = std::upper_bound(reference_fractions.begin(), reference_fractions.end(), fraction) -
                       reference_fractions.begin();
            // decrease index for convenience
            idx -= 1;
            if (idx == reference_fractions.size() - 1) {
                idx -= 1;
            }
            double fraction_lower = reference_fractions[idx];
            double fraction_upper = reference_fractions[idx + 1];
            double fraction_range = fraction_upper - fraction_lower;
            // check if fraction is same as lower or upper bound to avoid division by zero
            double tol = 1e-6;
            if (abs(fraction_lower - fraction) < tol) {
                discretized_points.push_back(reference_points[idx]);
            } else if (abs(fraction_upper - fraction) < tol) {
                discretized_points.push_back(reference_points[idx + 1]);
            } else {
                // get weighted point
                auto discretized_point =
                    (reference_points[idx] * (1.0 - (fraction - fraction_lower) / fraction_range) +
                     reference_points[idx + 1] * (1.0 - (fraction_upper - fraction) / fraction_range));
                discretized_points.push_back(discretized_point);
            }
        }
        return discretized_points;
    }
}

}  // namespace seahowl
