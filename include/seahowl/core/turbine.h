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
#include "seahowl/core/floater.h"
#include "seahowl/core/rotor.h"
#include "seahowl/core/tower.h"

// Standard library
#include <vector>

// forward declarations
namespace seahowl {
namespace env {
class SoilModel;
class FluidModel;
}  // namespace env
namespace servo {
class Controller;
}  // namespace servo
namespace fluid {
class TurbineFluid;
}  // namespace fluid
namespace elasto {
class TurbineElasto;
}  // namespace elasto
}  // namespace seahowl

namespace seahowl {
namespace core {

/**
 * @brief Wind turbine (blades, rotor-nacelle assembly, tower).
 *
 * This class controls each component, ensuring proper workflow and communication within and between the component.
 */
class Turbine : public ComponentDynamic {
  public:
    // components
    //
    /** @brief Elastodynamic model of the turbine. */
    seahowl::elasto::TurbineElasto& elasto;
    /** @brief Fluid model of the turbine. */
    seahowl::fluid::TurbineFluid& fluid;
    /** @brief Rotor-nacelle assembly of the turbine. */
    RotorNacelleAssembly rna;
    /** @brief Tower of the turbine. */
    Tower tower;
    /** @brief Controller of the turbine. */
    std::shared_ptr<seahowl::servo::Controller> controller;
    /** @brief Foundation of the turbine. */
    std::shared_ptr<Foundation> foundation;

    // parameters
    //
    /** @brief Efficiency of the generator. */
    double generator_efficiency = 1.0;
    /** @brief Ratio of the gearbox. */
    double gearbox_ratio = 1.0;
    /** @brief Efficiency of the gearbox. */
    double gearbox_efficiency = 1.0;

    /**
     * @brief Constructor.
     *
     * Instantiates turbine for communication between elasto and aero components.
     *
     * @param[in] elasto Elastodynamic turbine model.
     * @param[in] aero Aerodynamic turbine model.
     */
    Turbine(std::shared_ptr<seahowl::elasto::TurbineElasto> elasto,
            std::shared_ptr<seahowl::fluid::TurbineFluid> fluid);

    /**
     * @brief Applies control to turbine.
     */
    void apply_control(double time, double dt);

    void prestep(double time, double dt) override;
    void poststep(double time, double dt) override;
    virtual void build() override;

    /**
     * @brief Returns shaft power [W]
     */
    double get_shaft_power() const;

    /**
     * @brief Returns generated power [W]
     */
    double get_generated_power() const;

    /**
     * @brief Returns generator RPM [rpm]
     */
    double get_generator_rpm() const;

    virtual void apply_env_model(seahowl::env::EnvModel& env_model, double time) override;
    virtual void apply_soil_model(seahowl::env::EnvModel& env_model, double time) override;

  protected:
    /**
     * @brief Initialize turbine, called before starting the simulation.
     *
     * Calls init for each of its components.
     *
     * @param[in] time Time of the simulation (usually 0 at init) [s]
     * @param[in] dt Time step length [s]
     */
    virtual void initialize_this(double time, double dt) override;
};

}  // namespace core
}  // namespace seahowl
