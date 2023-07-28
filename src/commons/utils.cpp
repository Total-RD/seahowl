#include "seahowl/commons/utils.h"

#include <string>

using namespace seahowl;

Grid1D::Grid1D(){};

Grid1D::Grid1D(std::vector<double> ticks) {
    set_ticks(ticks);
}

void Grid1D::set_ticks(std::vector<double> ticks) {
    this->ticks = ticks;
}

int Grid1D::get_index(double value) {
    // get nearest-above
    auto idx0 = lower_bound(ticks.begin(), ticks.end(), value);
    // get nearest-above index
    int idx_below = int(idx0 - ticks.begin()) - 1;  // Nearest index
    return idx_below;
}

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
