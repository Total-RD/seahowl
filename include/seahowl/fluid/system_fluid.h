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
#include "seahowl/fluid/component_fluid.h"
#include "seahowl/fluid/turbine_fluid.h"

// Standard library
#include <deque>
#include <vector>

namespace seahowl {
namespace fluid {

/**
 * @brief Fluid system base class.
 */
class SystemFluid : public ComponentFluid {
  public:
    /** @brief Turbines in system. */
    std::deque<std::shared_ptr<TurbineFluid>> turbines{};
    /** @brief Components in system. */
    std::deque<std::shared_ptr<ComponentFluid>> components{};

    /**
     * @brief Builds the system.
     */
    void build() override;

    /**
     * @brief Computes fluid loads on system.
     *
     * @param[in] env_model env model to use for applying fluid loads.
     * @param[in] time Time of simulation [s]
     */
    void compute_env_loads(const env::EnvModel& env_model, double time) override;

    /**
     * @brief Adds turbine to system.
     *
     * @param[in] turbine Turbine to add to system.
     */
    void add(std::shared_ptr<TurbineFluid> turbine);

    /**
     * @brief Adds component to system.
     *
     * @param[in] component Component to add to system.
     */
    void add(std::shared_ptr<seahowl::fluid::ComponentFluid> component);
};

// Backward-compatible alias
using SystemAero = SystemFluid;

}  // namespace fluid
}  // namespace seahowl
