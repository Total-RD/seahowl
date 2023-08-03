#include "seahowl/commons/utils.h"

#include <string>
#include <iostream>

std::vector<seahowl::DiscretizationPoint> seahowl::get_indice_and_positions(
    const std::vector<double>& discretization_fractions,
    const std::vector<double>& reference_fractions) {
    // check for potential errors
    if (discretization_fractions.size() == 0) {
        throw std::runtime_error("Cannot get discretization with empty array.");
    } else if (reference_fractions.size() < 2) {
        throw std::runtime_error("Cannot get discretization with reference array with less than 2 elements.");
    }

    std::vector<DiscretizationPoint> points;
    for (int ii = 0; ii < discretization_fractions.size(); ii++) {
        double fraction = discretization_fractions[ii];
        // check bounds
        if (fraction < 0.0 || fraction > 1.0) {
            throw std::runtime_error("Discretization fraction must be between 0 and 1 but was " +
                                     std::to_string(fraction) + ".");
        }
        if (fraction == 0) {
            DiscretizationPoint point{};
            point.index = 0;
            point.eta = -1;
            points.push_back(point);
        } else {
            auto idx = std::upper_bound(reference_fractions.begin(), reference_fractions.end(), fraction) -
                       reference_fractions.begin();
            // decrease index for getting lower bound
            idx -= 1;
            if (idx == reference_fractions.size() - 1) {
                idx -= 1;
            }
            double fraction_lower = reference_fractions[idx];
            double fraction_upper = reference_fractions[idx + 1];
            double fraction_range = fraction_upper - fraction_lower;
            double eta = 2.0 * (fraction - fraction_lower) / fraction_range - 1.0;
            DiscretizationPoint point{};
            point.index = static_cast<decltype(point.index)>(idx);
            point.eta = eta;
            points.push_back(point);
        }
    }
    return points;
}

double seahowl::bilinear_interpolation(const Eigen::MatrixXd& dataMatrix,
                                       const Eigen::VectorXd& x_list,
                                       const Eigen::VectorXd& y_list,
                                       double x,
                                       double y) {
    // Find the four surrounding data points
    int x0, y0 = -99;
    for (unsigned ii = 0; ii < x_list.size() - 1; ii++) {
        if (x >= x_list[ii] && x <= x_list[ii + 1])
            x0 = ii;
    }
    for (unsigned ii = 0; ii < y_list.size() - 1; ii++) {
        if (y >= y_list[ii] && y <= y_list[ii + 1])
            y0 = ii;
    }
    if (y0 == -99 || x0 == -99) {
        std::cout << "x = " << x << ",  y = " << y << " , " << std::endl;
        std::cout << "x list = " << x_list << ",  y list = " << y_list << " , " << std::endl;
        throw std::runtime_error("x or y not found in the coefficients list.");
    };

    int x1 = x0 + 1;
    int y1 = y0 + 1;

    double x_interp = x0 + (x - x_list[x0]) / (x_list[x1] - x_list[x0]);
    double y_interp = y0 + (y - y_list[y0]) / (y_list[y1] - y_list[y0]);

    double fQ11 = dataMatrix(y0, x0);
    double fQ21 = dataMatrix(y0, x1);
    double fQ12 = dataMatrix(y1, x0);
    double fQ22 = dataMatrix(y1, x1);

    double fR1 = (x1 - x_interp) / (x1 - x0) * fQ11 + (x_interp - x0) / (x1 - x0) * fQ21;
    double fR2 = (x1 - x_interp) / (x1 - x0) * fQ12 + (x_interp - x0) / (x1 - x0) * fQ22;

    double fP = (y1 - y_interp) / (y1 - y0) * fR1 + (y_interp - y0) / (y1 - y0) * fR2;

    return fP;
}
