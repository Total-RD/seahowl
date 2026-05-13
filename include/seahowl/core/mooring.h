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
#include "seahowl/commons/utils.h"  // for DiscretizationPoint
#include "seahowl/core/component.h"
#include "seahowl/core/component_elasto_fluid.h"

// Standard library
#include <deque>
#include <memory>
#include <vector>

// forward declarations
namespace seahowl {
namespace elasto {
class MooringElastoFEA;
class MooringSystemElasto;
}  // namespace elasto
namespace fluid {
namespace hydro {
class MooringHydro;
class MooringSystemHydro;
}  // namespace hydro
}  // namespace fluid
}  // namespace seahowl

namespace seahowl {
namespace core {

/**
 * @brief Mooring of wind turbine, with both elasto and hydro components.
 *
 * This class acts as a "mediator" between the elasto and hydro components.
 * Mappings between elasto nodes to hydro domain and hydro nodes to elasto domain are used to ensure communication
 * between the hydro and elasto components. The hydro loads are communicated to the elasto component in the prestep,
 * while the hydro positions are updated using the elasto positions in the poststep.
 */
class Mooring : public ComponentElastoFluid {
  public:
    /** @brief Elastodynamic model of the mooring. */
    seahowl::elasto::MooringElastoFEA& elasto;
    /** @brief Hydrodynamic model of the mooring. */
    seahowl::fluid::hydro::MooringHydro& hydro;

    /**
     * @brief Instantiates mooring for communication between elasto and hydro components.
     *
     * @param[in] elasto Elastodynamic mooring model.
     * @param[in] hydro hydrodynamic mooring model.
     */
    Mooring(std::shared_ptr<seahowl::elasto::MooringElastoFEA> elasto,
            std::shared_ptr<seahowl::fluid::hydro::MooringHydro> hydro);

    /**
     * @brief Sets length of the mooring line.
     *
     * @param[in] length Length of the mooring line [m]
     */
    void set_length(double length);

    /**
     * @brief Sets diameter of the mooring line.
     *
     * @param[in] diameter Diameter of the mooring line [m]
     */
    void set_diameter(double diameter);

    void prestep(double time, double dt) override;
    void poststep(double time, double dt) override;
    void apply_env_model(seahowl::env::EnvModel& env_model, double time) override;
    void apply_soil_model(seahowl::env::EnvModel& soil_model, double time) override;
    void build() override;

    /**
     * @brief Sets the discretization fractions to use when building the elasto part of the mooring.
     *
     * @param[in] fractions Normalized discretization fractions within [0, 1].
     */
    void set_discretization_elasto(const std::vector<double>& fractions);

    /**
     * @brief Sets the discretization fractions to use when building the hydro part of the mooring.
     *
     * @param[in] fractions Normalized discretization fractions within [0, 1].
     */
    void set_discretization_hydro(const std::vector<double>& fractions);

    /**
     * @brief Updates hydro positions, rotations, velocities and accelerations from elasto component of the mooring.
     */
    void update_positions_hydro();

    /**
     * @brief Accumulates hydro loads to the elasto component of the mooring.
     */
    void update_loads_elasto();

  private:
    void perform_sanity_check();

    /**
     * @brief Initialize mooring, called before starting the simulation.
     *
     * Runs the preset and poststep once to make elasto and hydro components match.
     *
     * @param[in] time Time of the simulation (usually 0 at init) [s]
     * @param[in] dt Time step length [s]
     */
    void initialize_this(double time, double dt) override;
};

class MooringSystem : public ComponentDynamic {
  public:
    /** @brief Elastodynamic model of the mooring system. */
    seahowl::elasto::MooringSystemElasto& elasto;
    /** @brief Hydrodynamic model of the mooring system. */
    seahowl::fluid::hydro::MooringSystemHydro& hydro;
    /** @brief List of mooring lines. */
    std::deque<std::shared_ptr<Mooring>> moorings;

    /**
     * @brief Constructor.
     *
     * @param[in] elasto Elastodynamic mooring system model.
     * @param[in] hydro hydrodynamic mooring system model.
     */
    MooringSystem(std::shared_ptr<seahowl::elasto::MooringSystemElasto> elasto,
                  std::shared_ptr<seahowl::fluid::hydro::MooringSystemHydro> hydro);

    /**
     * @brief Adds mooring to mooring system.
     *
     * @param[in] mooring Mooring to add to mooring system.
     */
    void add_mooring(std::shared_ptr<Mooring> mooring);

    void prestep(double time, double dt) override;
    void poststep(double time, double dt) override;
    void apply_env_model(seahowl::env::EnvModel& env_model, double time) override;
    void apply_soil_model(seahowl::env::EnvModel& env_model, double time) override;
    void build() override;

  private:
    void perform_sanity_check();
    void initialize_this(double time, double dt) override;
};

}  // namespace core
}  // namespace seahowl
