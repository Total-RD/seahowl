#include "seahowl/commons/utils.h"

#include <string>
#include <spdlog/spdlog.h>
#include <spdlog/fmt/ostr.h>

void seahowl::set_log_level_global(std::string level) {
    if (level == "critical") {
        spdlog::set_level(spdlog::level::critical);
    } else if (level == "error" || level == "err") {
        spdlog::set_level(spdlog::level::err);
    } else if (level == "warning" || level == "warn") {
        spdlog::set_level(spdlog::level::warn);
    } else if (level == "info") {
        spdlog::set_level(spdlog::level::info);
    } else if (level == "debug") {
        spdlog::set_level(spdlog::level::debug);
    } else if (level == "trace") {
        spdlog::set_level(spdlog::level::trace);
    }
}

std::vector<seahowl::DiscretizationPoint> seahowl::get_indice_and_positions(
    const std::vector<double>& discretization_fractions,
    const std::vector<double>& reference_fractions) {
    // check for potential errors
    if (discretization_fractions.size() == 0) {
        spdlog::error("Cannot get discretization with empty array.");
        exit(1);
    } else if (reference_fractions.size() < 2) {
        spdlog::error("Cannot get discretization with reference array with less than 2 elements.");
        exit(1);
    }

    std::vector<DiscretizationPoint> points;
    for (int ii = 0; ii < discretization_fractions.size(); ii++) {
        double fraction = discretization_fractions[ii];
        // check bounds
        if (fraction < 0.0 || fraction > 1.0) {
            spdlog::error("Discretization fraction must be between 0 and 1 but was {}.", fraction);
            exit(1);
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
                                       double y,
                                       double x) {
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
        spdlog::error("Bilinear interpolation failed: x or y not found in the coefficients list.");
        spdlog::error("    x = {}, y = {},", x, y);
        spdlog::error("    x_list = {},", x_list.transpose());
        spdlog::error("    y_list = {}.", y_list.transpose());
        exit(1);
    };

    double x_frac = (x - x_list[x0]) / (std::fabs(x_list[x0] - x_list[x0 + 1]));
    double y_frac = (y - y_list[y0]) / (std::fabs(y_list[y0] - y_list[y0 + 1]));

    double q11 = dataMatrix(y0, x0);
    double q21 = dataMatrix(y0, x0 + 1);
    double q12 = dataMatrix(y0 + 1, x0);
    double q22 = dataMatrix(y0 + 1, x0 + 1);

    double fP = (1 - x_frac) * (1 - y_frac) * q11 + x_frac * (1 - y_frac) * q21 + (1 - x_frac) * y_frac * q12 +
                x_frac * y_frac * q22;

    return fP;
}
