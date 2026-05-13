// SPDX-License-Identifier: Apache-2.0
// Copyright 2022–2026 TotalEnergies
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//

#include "seahowl/fluid/hydro/floater_hydro.h"

// SEAHOWL headers
#include "seahowl/env/env_model.h"
#include "seahowl/fluid/hydro/mooring_hydro.h"

using namespace seahowl;
using namespace seahowl::fluid::hydro;
using namespace seahowl::env;

FloaterHydro::FloaterHydro() : mooring_system(std::make_unique<MooringSystemHydro>()) {
    body_main = std::make_unique<seahowl::EntityDynamicEigen>();
}

void FloaterHydro::build() {
    mooring_system->build();
}

void FloaterHydro::compute_env_loads(const EnvModel& env_model, double time) {
    mooring_system->compute_env_loads(env_model, time);
}

Vector3d FloaterHydro::get_force_hydro() {
    return force_hydro;
}

Vector3d FloaterHydro::get_torque_hydro() {
    return torque_hydro;
}

Eigen::Matrix<double, 6, 6> FloaterHydro::get_added_mass_matrix() {
    return added_mass_matrix;
}
