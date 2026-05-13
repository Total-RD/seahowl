// SPDX-License-Identifier: Apache-2.0
// Copyright 2022–2026 TotalEnergies
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//

#pragma once

// Third-party libraries
#include <Eigen/Dense>

// Standard library
#include <algorithm>
#include <stdexcept>
#include <vector>

namespace seahowl {

extern bool LOG_LEVEL_SET;

/**
 * @brief Set global logging level.
 *
 * @param[in] level Logging level ("critical", "error", "warning", "info", "debug", "trace").
 */
void set_log_level_global(const std::string& level);

/**
 * @brief Logs a message.
 *
 * @param[in] message Message to log.
 * @param[in] level Log level of message ("critical", "error", "warning", "info", "debug", "trace").
 */
void log(const std::string& message, const std::string& level);

/**
 * @brief Parametric discretization point.
 */
struct DiscretizationPoint {
    /** @brief Index of element. */
    int index = 0;
    /** @brief Abscissa along element within range [-1, +1], with -1 at node1, and +1 at node2. */
    double eta = 0;
};

/**
 * @brief Computes discretization points indices and positions from discretization and reference fractions.
 *
 * Maps discretization fractions to reference fractions, returning the element index and
 * the normalized position (eta) within each element.
 *
 * @param[in] discretization_fractions Fractions at which to discretize, within [0, 1].
 * @param[in] reference_fractions Reference fractions defining element boundaries, within [0, 1].
 * @return Vector of DiscretizationPoint containing element index and eta position.
 */
std::vector<DiscretizationPoint> get_indice_and_positions(const std::vector<double>& discretization_fractions,
                                                          const std::vector<double>& reference_fractions);

/**
 * @brief Interpolates reference points at specified discretization fractions.
 *
 * If discretization_fractions is empty, returns reference_points directly.
 * Otherwise, interpolates between reference points to create discretized points
 * at the specified fractions. Reference points must have a 'fraction' member.
 *
 * @tparam T Type of reference point (must have 'fraction' member and support arithmetic operators).
 * @param[in] discretization_fractions Fractions at which to interpolate, within [0, 1].
 * @param[in] reference_points Reference points with fractions starting at 0 and ending at 1.
 * @return Vector of interpolated points at the discretization fractions.
 */
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

/**
 * @brief Performs bilinear interpolation on a 2D data matrix.
 *
 * Interpolates a value from a 2D data matrix at coordinates (x, y) using
 * the four nearest grid points.
 *
 * @param[in] dataMatrix 2D matrix of data values.
 * @param[in] x_list Vector of x-coordinates corresponding to matrix columns.
 * @param[in] y_list Vector of y-coordinates corresponding to matrix rows.
 * @param[in] x X-coordinate at which to interpolate.
 * @param[in] y Y-coordinate at which to interpolate.
 * @return Interpolated value at (x, y).
 */
double bilinear_interpolation(const Eigen::MatrixXd& dataMatrix,
                              const Eigen::VectorXd& x_list,
                              const Eigen::VectorXd& y_list,
                              double x,
                              double y);

}  // namespace seahowl
