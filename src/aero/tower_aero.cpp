#include "seahowl/aero/tower_aero.h"

#include "seahowl/commons/utils.h"
#include "seahowl/commons/numerics.h"
#include "seahowl/aero/reference_point_aero.h"
#include "seahowl/env/wind_models.h"

#include <spdlog/spdlog.h>

using namespace seahowl;
using namespace seahowl::aero;
using namespace seahowl::hydro;
using seahowl::env::FluidModel;

TowerAero::TowerAero() {}

void TowerAero::build() {
    // check that enough reference points were defined to create elements (at least 2)
    if (reference_points.size() < 2) {
        throw std::runtime_error("Not enough aero reference points defined for tower (" +
                                 std::to_string(reference_points.size()) + ").");
    }

    // check that discretization_fractions was defined, otherwise take reference point fractions
    if (discretization_fractions.size() == 0) {
        for (int ii = 0; ii < reference_points.size(); ii++) {
            discretization_fractions.push_back(reference_points[ii].fraction);
        }
    } else if (discretization_fractions.size() == 1) {
        double npoints = discretization_fractions[0] + 1;
        double dp = 1.0 / (npoints - 1);
        discretization_fractions.clear();
        for (int ii = 0; ii < int(npoints); ii++) {
            discretization_fractions.push_back(ii * dp);
        }
    }

    // build
    discretized_points = seahowl::get_discretized_points(discretization_fractions, reference_points);
    // nodes
    nodes.clear();
    for (auto& point : discretized_points) {
        // push empty load
        nodes.push_back(MorisonNode());
        nodes.back().set_position(point.coordinates);
        nodes.back().diameter = point.diameter;
        nodes.back().coefficients.drag_normal = point.drag_coefficient;
    }
    // elements
    elements.clear();
    loads.clear();
    for (int ii = 0; ii < discretized_points.size() - 1; ii++) {
        elements.push_back(MorisonElement(nodes[ii], nodes[ii + 1]));
        loads.push_back(Vector3d(0.0, 0.0, 0.0));
    }
}

void TowerAero::compute_aero_loads(const FluidModel& wind_model, double time) {
    // compute loads at nodes
    for (auto& node : nodes) {
        node.compute_loads(wind_model, time);
    }

    // integrate loads over elements and store them
    for (int ii = 0; ii < elements.size(); ii++) {
        loads[ii] = elements[ii].get_load();
    }
}
