// SPDX-License-Identifier: Apache-2.0
// Copyright 2022–2026 TotalEnergies
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//

#include "seahowl/fluid/aero/turbine_aero.h"

using namespace seahowl::aero;
using seahowl::env::EnvModel;

TurbineAero::TurbineAero() {
    rna = std::make_shared<RotorNacelleAssemblyAero>();
    tower = std::make_shared<TowerAero>();
}

void TurbineAero::build() {
    rna->build();
    tower->build();
    if (foundation) {
        foundation->build();
    }
}

void TurbineAero::setup_environment(const env::EnvModel& env_model) {
    rna->setup_environment(env_model);
    tower->setup_environment(env_model);
    if (foundation) {
        foundation->setup_environment(env_model);
    }
}

void TurbineAero::initialize(double time, double dt) {}

void TurbineAero::compute_env_loads(const EnvModel& env_model, double time) {
    rna->compute_env_loads(env_model, time);
    tower->compute_env_loads(env_model, time);
    if (foundation) {
        foundation->compute_env_loads(env_model, time);
    }
}
