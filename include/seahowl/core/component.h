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
#include "seahowl/commons/entities.h"

// Standard library
#include <algorithm>
#include <memory>
#include <stdexcept>
#include <vector>

// forward declarations
namespace seahowl {
namespace fluid {
class ComponentFluid;
}  // namespace fluid
namespace env {
class EnvModel;
}  // namespace env
namespace elasto {
class ComponentElasto;
}
}  // namespace seahowl

namespace seahowl {
/** @brief Core module for dynamic components and simulation workflow. */
namespace core {

/**
 * @brief Component that evolves dynamically during simulation.
 */
class ComponentDynamic {
  public:
    ComponentDynamic(const std::shared_ptr<seahowl::elasto::ComponentElasto> elasto_,
                     const std::shared_ptr<seahowl::fluid::ComponentFluid> fluid_)
        : elasto_ptr(elasto_), fluid_ptr(fluid_) {}
    /**
     * @brief Virtual destructor.
     */
    virtual ~ComponentDynamic() = default;
    /**
     * @brief Builds the component, called before initializing the simulation.
     */
    virtual void build() = 0;

    /**
     * @brief Initializes the component, called before starting the simulation.
     *
     * @param[in] time Time of the simulation (usually 0 at init) [s]
     * @param[in] dt Time step length [s]
     */
    void initialize(double time, double dt);

    /**
     * @brief Prestep for component, called before elastodynamic stepping.
     *
     * @param[in] time Time of the simulation [s]
     * @param[in] dt Time step length [s]
     */
    virtual void prestep(double time, double dt) = 0;

    /**
     * @brief Poststep for component, called after elastodynamic stepping.
     *
     * @param[in] time Time of the simulation [s]
     * @param[in] dt Time step length [s]
     */
    virtual void poststep(double time, double dt) = 0;

    /**
     * @brief Applies env model to component.
     *
     * @param[in] env_model env model affecting component.
     * @param[in] time Time of simulation [s]
     */
    virtual void apply_env_model(seahowl::env::EnvModel& env_model, double time){};

    /**
     * @brief Applies soil model to component.
     *
     * @param[in] env_model env model affecting component.
     * @param[in] time Time of simulation [s]
     */
    virtual void apply_soil_model(seahowl::env::EnvModel& env_model, double time){};

    /**
     * @brief Get elasto shared_ptr component.
     */
    std::shared_ptr<seahowl::elasto::ComponentElasto> get_shared_elasto() const { return elasto_ptr; }

    /**
     * @brief Get fluid shared_ptr component.
     */
    std::shared_ptr<seahowl::fluid::ComponentFluid> get_shared_fluid() const { return fluid_ptr; }

  protected:
    bool is_initialized = false;

  private:
    // Only for memory management, never accessed (reference to underlying object is accessed instead).
    std::shared_ptr<seahowl::elasto::ComponentElasto> elasto_ptr;
    // Only for memory management, never accessed (reference to underlying object is accessed instead).
    std::shared_ptr<seahowl::fluid::ComponentFluid> fluid_ptr;

    /**
     * @brief Component-specific initialization, called by initialize().
     *
     * @param[in] time Time of the simulation (usually 0 at init) [s]
     * @param[in] dt Time step length [s]
     */
    virtual void initialize_this(double time, double dt) = 0;
};

}  // namespace core
}  // namespace seahowl
