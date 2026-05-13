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
#include "seahowl/fluid/aero/rotor_aero.h"
#include "seahowl/fluid/aero/tower_aero.h"
#include "seahowl/fluid/hydro/foundation_fluid.h"
#include "seahowl/fluid/hydro/floater_hydro.h"
#include "seahowl/fluid/component_fluid.h"

// Standard library
#include <vector>

// forward declarations
namespace seahowl {
namespace env {
class WindModel;
}  // namespace env
}  // namespace seahowl

namespace seahowl {
namespace fluid {

/**
 * @brief Wind turbine (blades, rotor-nacelle assembly, tower).
 *
 * This class controls each component, ensuring proper workflow for the fluid part.
 */
class TurbineFluid : public ComponentFluid {
  public:
    // components
    //
    /** @brief Rotor-nacelle assembly of the turbine. */
    std::shared_ptr<seahowl::fluid::aero::RotorNacelleAssemblyAero> rna;
    /** @brief Tower of the turbine. */
    std::shared_ptr<seahowl::fluid::aero::TowerAero> tower;
    /** @brief Foundation of the turbine. */
    std::shared_ptr<seahowl::fluid::hydro::FoundationFluid> foundation;

    /**
     * @brief Constructor.
     *
     * Instantiates rotor component and tower component.
     */
    TurbineFluid();

    void setup_environment(const env::EnvModel& env_model) override;

    /**
     * @brief Builds turbine.
     */
    void build() override;

    /**
     * @brief Initializes turbine.
     *
     * @param[in] time Time of the simulation (usually 0 at init) [s]
     * @param[in] dt Time step length [s]
     */
    virtual void initialize(double time, double dt);

    /**
     * @brief Computes fluid loads on turbine.
     *
     * @param[in] env_model Environment model to use for applying fluid loads.
     * @param[in] time Time of simulation [s]
     */
    virtual void compute_env_loads(const env::EnvModel& env_model, double time) override;
};

// Backward-compatible alias
using TurbineAero = TurbineFluid;

}  // namespace fluid
}  // namespace seahowl
