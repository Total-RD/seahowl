// SPDX-License-Identifier: Apache-2.0
// Copyright 2022–2026 TotalEnergies
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//

#include "seahowl/core/floater.h"

// SEAHOWL headers
#include "seahowl/core/mooring.h"
#include "seahowl/elasto/floater_elasto.h"
#include "seahowl/elasto/mooring_elasto.h"
#include "seahowl/fluid/hydro/floater_hydro.h"
#include "seahowl/fluid/hydro/mooring_hydro.h"

// Third-party libraries
#include <spdlog/spdlog.h>

// Standard library
#include <memory>
#include <vector>

using namespace seahowl::core;
using namespace seahowl::elasto;
using namespace seahowl::fluid::hydro;

Floater::Floater(std::shared_ptr<seahowl::elasto::FloaterElasto> elasto,
                 std::shared_ptr<seahowl::hydro::FloaterHydro> hydro)
    : Foundation(elasto, hydro),
      ComponentDynamic(elasto, hydro),
      elasto(*elasto),
      hydro(*hydro),
      mooring_system(std::make_unique<MooringSystem>(elasto->mooring_system, hydro->mooring_system)) {}

Floater::~Floater() = default;

void Floater::initialize_this(double time, double dt) {
    mooring_system->initialize(time, dt);

    elasto.initialize();
    // update positions before initializing hydro part
    // needed for initial positions for HydroDyn
    update_positions_hydro();
    hydro.initialize(time, dt);

    spdlog::info("Initialized floater of total mass {:.4}kg (with moorings).", elasto.get_mass());
}

void Floater::prestep(double time, double dt) {
    mooring_system->prestep(time, dt);

    update_loads_elasto();
}

void Floater::poststep(double time, double dt) {
    mooring_system->poststep(time, dt);

    update_positions_hydro();
}

void Floater::apply_env_model(seahowl::env::EnvModel& env_model, double time) {
    mooring_system->apply_env_model(env_model, time);
}

void Floater::apply_soil_model(seahowl::env::EnvModel& env_model, double time) {
    mooring_system->apply_soil_model(env_model, time);
}

void Floater::build() {
    // build elasto
    elasto.build();
}

void Floater::update_positions_hydro() {
    // main body
    hydro.body_main->set_rotation(elasto.body_main->get_rotation());
    hydro.body_main->set_position(elasto.body_main->get_position());
    hydro.body_main->set_velocity(elasto.body_main->get_velocity());
    hydro.body_main->set_rotational_velocity(elasto.body_main->get_rotational_velocity());
    hydro.body_main->set_acceleration(elasto.body_main->get_acceleration());
    hydro.body_main->set_rotational_acceleration(elasto.body_main->get_rotational_acceleration());
}

void Floater::update_loads_elasto() {
    elasto.body_main->reset_loads_internals();

    // set hydro forces if any
    elasto.body_main->accumulate_force_internals(hydro.get_force_hydro(), false);
    elasto.body_main->accumulate_torque_internals(hydro.get_torque_hydro(), false);
    elasto.body_main->set_added_mass_matrix(hydro.get_added_mass_matrix());
}
