// SPDX-License-Identifier: Apache-2.0
// Copyright 2022–2026 TotalEnergies
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//

#include "seahowl/fluid/aero/tower_aero.h"

// SEAHOWL headers
#include "seahowl/commons/numerics.h"
#include "seahowl/commons/utils.h"
#include "seahowl/env/env_model.h"
#include "seahowl/env/wind_models.h"

// Third-party libraries
#include <spdlog/spdlog.h>

using namespace seahowl;
using namespace seahowl::fluid::aero;
using namespace seahowl::fluid::hydro;
using seahowl::env::EnvModel;

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
    for (const auto& point : discretized_points) {
        // push empty load
        nodes.push_back(MorisonNode());
        nodes.back().set_position(point.coordinates);
        nodes.back().diameter = point.diameter;
        nodes.back().coefficients = point.coefficients;
        nodes.back().coefficients.use_MacCamyFuchs_correction = use_MacCamyFuchs_correction;
        nodes.back().coefficients.use_Cd_correction = use_Cd_correction;
    }
    // elements
    elements.clear();
    for (int ii = 0; ii < discretized_points.size() - 1; ii++) {
        elements.push_back(MorisonElement(nodes[ii], nodes[ii + 1]));
    }
}

void TowerAero::compute_env_loads(const EnvModel& wind_model, double time) {
    // compute loads at nodes
    for (auto& node : nodes) {
        node.compute_env_loads(wind_model, time);
    }
}
