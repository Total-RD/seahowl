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
#include "seahowl/core/component.h"
#include "seahowl/core/turbine.h"

// Standard library
#include <deque>

// Forward declarations
namespace seahowl {
namespace env {
class EnvModel;
}  // namespace env
namespace servo {
class Controller;
}  // namespace servo
namespace fluid {
class SystemFluid;
}  // namespace fluid
namespace elasto {
class SystemElasto;
}  // namespace elasto
}  // namespace seahowl

namespace seahowl {
namespace core {

/**
 * @brief System composed of wind turbines and environmental conditions.
 *
 * This class controls the workflow between wind turbines and the environment (wind, waves, etc).
 */
class System : public ComponentDynamic {
  public:
    /** @brief Wind turbines. */
    std::deque<std::shared_ptr<Turbine>> turbines{};
    /** @brief Other dynamics components. */
    std::deque<std::shared_ptr<ComponentDynamic>> components{};
    /** @brief Environmental model. */
    std::shared_ptr<seahowl::env::EnvModel> env_model;
    /** @brief System for elastodynamics. */
    seahowl::elasto::SystemElasto& elasto;
    /** @brief System for fluid dynamics. */
    seahowl::fluid::SystemFluid& fluid;
    /**
     * @brief Constructor.
     *
     * Instantiates system for communication between elasto, aero, servo, hydro components.
     *
     * @param[in] elasto Elastodynamic system.
     * @param[in] aero Aerodynamic system.
     */
    System(std::shared_ptr<seahowl::elasto::SystemElasto> elasto, std::shared_ptr<seahowl::fluid::SystemFluid> fluid);

    void build() override;
    virtual void prestep(double time, double dt) override;

    /**
     * @brief Step for system, called for elastodynamic stepping.
     *
     * @param[in] time Time of the simulation [s]
     */
    void step(double dt);

    virtual void poststep(double time, double dt) override;
    void apply_env_model(seahowl::env::EnvModel& env_model, double time) override;
    void apply_soil_model(seahowl::env::EnvModel& env_model, double time) override;

    /**
     * @brief Returns time of simulation [s]
     */
    double get_time() const;

    /**
     * @brief Sets time of simulation.
     *
     * @param[in] time Time of simulation [s]
     */
    void set_time(double time);

    /**
     * @brief Presimulation for system, called before simulation actually starts.
     *
     * @param[in] duration Duration of presimulation [s]
     * @param[in] dt Time step length [s]
     * @param[in] fix_towers Whether to fix tower bases or not.
     * @param[in] with_presetup Whether to do presetup or not.
     */
    void run_presimulation(double duration, double dt, bool fix_towers = true, bool with_presetup = true);

    /**
     * @brief Adds turbine to system.
     *
     * @param[in] turbine Turbine to add to system.
     */
    void add(std::shared_ptr<seahowl::core::Turbine> turbine);

    /**
     * @brief Adds component to system.
     *
     * @param[in] component Component to add to system.
     */
    void add(std::shared_ptr<seahowl::core::ComponentDynamic> component);

  private:
    /**
     * @brief Initialize system.
     *
     * @param[in] time Time of the simulation (usually 0 at init) [s]
     * @param[in] dt Time step length [s]
     */
    virtual void initialize_this(double time, double dt) override;
};
}  // namespace core
}  // namespace seahowl
