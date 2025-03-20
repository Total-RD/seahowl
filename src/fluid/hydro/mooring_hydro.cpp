#include "seahowl/fluid/hydro/mooring_hydro.h"

#include "seahowl/commons/utils.h"
#include "seahowl/commons/numerics.h"
#include "seahowl/env/env_model.h"

#include <spdlog/spdlog.h>

using namespace seahowl;
using namespace seahowl::hydro;
using seahowl::env::EnvModel;

MooringHydro::MooringHydro() {
    coefficients.use_MacCamyFuchs_correction = false;
    coefficients.use_Cd_correction = false;
}

void MooringHydro::set_length(double length) {
    this->length = length;
    for (int ii = 0; ii < elements.size(); ii++) {
        elements[ii].length = length * abs(discretization_fractions[ii + 1] - discretization_fractions[ii]);
    }
}

void MooringHydro::set_diameter(double diameter) {
    this->diameter = diameter;
    for (auto& node : nodes) {
        node.diameter = diameter;
    }
}

void MooringHydro::build() {
    // check that discretization_fractions was defined, otherwise take reference point fractions
    if (discretization_fractions.size() == 0) {
        throw std::runtime_error("Mooring hydro discretization was not defined.");
    } else if (discretization_fractions.size() == 1) {
        double npoints = discretization_fractions[0] + 1;
        double dp = 1.0 / (npoints - 1);
        discretization_fractions.clear();
        for (int ii = 0; ii < int(npoints); ii++) {
            discretization_fractions.push_back(ii * dp);
        }
    }

    // nodes
    nodes.clear();
    for (auto& fraction : discretization_fractions) {
        // push empty load
        nodes.push_back(MorisonNode());
        nodes.back().diameter = diameter;
        nodes.back().coefficients = coefficients;
    }
    // elements
    elements.clear();
    for (int ii = 0; ii < discretization_fractions.size() - 1; ii++) {
        elements.push_back(MorisonElement(nodes[ii], nodes[ii + 1]));
        elements.back().length = length * abs(discretization_fractions[ii + 1] - discretization_fractions[ii]);
    }
}

void MooringHydro::compute_env_loads(const EnvModel& env_model, double time) {
    // compute loads at nodes
    for (auto& node : nodes) {
        node.compute_env_loads(env_model, time);
    }
}

MooringSystemHydro::MooringSystemHydro() {}

void MooringSystemHydro::add_mooring(std::shared_ptr<MooringHydro> mooring) {
    moorings.push_back(mooring);
}

void MooringSystemHydro::build() {
    for (auto& mooring : moorings) {
        mooring->build();
    }
}

void MooringSystemHydro::compute_env_loads(const EnvModel& env_model, double time) {
    for (auto& mooring : moorings) {
        mooring->compute_env_loads(env_model, time);
    }
}
