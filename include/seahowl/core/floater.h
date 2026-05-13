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

// SEAHOWL headers
#include "seahowl/core/foundation.h"

// Standard library
#include <memory>
#include <vector>

// forward declarations
namespace seahowl {
namespace core {
class MooringSystem;
}  // namespace core
namespace elasto {
class FloaterElasto;
}  // namespace elasto
namespace fluid {
namespace hydro {
class FloaterHydro;
}  // namespace hydro
}  // namespace fluid
}  // namespace seahowl

namespace seahowl {
namespace core {

/**
 * @brief Floater of wind turbine, with both elasto and hydro components.
 *
 */
class Floater : public Foundation {
  public:
    /** @brief Elastodynamic model of the mooring. */
    seahowl::elasto::FloaterElasto& elasto;
    /** @brief Hydrodynamic model of the mooring. */
    seahowl::fluid::hydro::FloaterHydro& hydro;
    /** @brief Mooring system of the floater. */
    std::unique_ptr<MooringSystem> mooring_system;

    /**
     * @brief Instantiates floater for communication between elasto and hydro components.
     *
     * @param[in] elasto Elastodynamic floater model.
     * @param[in] aero Hydrodynamic floater model.
     */
    Floater(std::shared_ptr<seahowl::elasto::FloaterElasto> elasto,
            std::shared_ptr<seahowl::fluid::hydro::FloaterHydro> hydro);

    /** @brief Destructor. */
    ~Floater();

    void prestep(double time, double dt) override;
    void poststep(double time, double dt) override;
    void apply_env_model(seahowl::env::EnvModel& env_model, double time) override;
    void apply_soil_model(seahowl::env::EnvModel& soil_model, double time) override;
    void build() override;

    /**
     * @brief Updates hydro positions, rotations, velocities and accelerations from elasto component of the blade.
     */
    void update_positions_hydro();

    /**
     * @brief Accumulates hydro loads to the elasto component of the blade.
     */
    void update_loads_elasto();

  private:
    /**
     * @brief Initialize floater, called before starting the simulation.
     *
     * Runs the preset and poststep once to make elasto and aero components match.
     *
     * @param[in] time Time of the simulation (usually 0 at init) [s]
     * @param[in] dt Time step length [s]
     */
    void initialize_this(double time, double dt) override;
};

}  // namespace core
}  // namespace seahowl
