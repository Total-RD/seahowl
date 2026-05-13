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
#include "seahowl/commons/utils.h"
#include "seahowl/core/component.h"
#include "seahowl/core/component_elasto_fluid.h"

// Standard library
#include <memory>
#include <vector>

// forward declarations
namespace seahowl {
struct DiscretizationPoint;
namespace core {
class Blade;
}  // namespace core
namespace elasto {
class BladeElasto;
}  // namespace elasto
namespace fluid {
namespace aero {
class BladeAero;
}  // namespace aero
}  // namespace fluid
}  // namespace seahowl

namespace seahowl {
namespace core {

/**
 * @brief Blade of wind turbine, with both elasto and aero components.
 *
 * This class acts as a "mediator" between the elasto and aero components.
 * Mappings between elasto nodes to aero domain and aero nodes to elasto domain are used to ensure communication between
 * the aero and elasto components. The aero loads are communicated to the elasto component in the prestep, while the
 * aero positions are updated using the elasto positions in the poststep.
 */
class Blade : public ComponentElastoFluid {
  public:
    /** @brief Elastodynamic model of the blade. */
    seahowl::elasto::BladeElasto& elasto;
    /** @brief Aerodynamic model of the blade. */
    seahowl::fluid::aero::BladeAero& aero;

    /**
     * @brief Instantiates blade for communication between elasto and aero components.
     *
     * @param[in] elasto Elastodynamic blade model.
     * @param[in] aero Aerodynamic blade model.
     */
    Blade(const std::shared_ptr<seahowl::elasto::BladeElasto> elasto,
          const std::shared_ptr<seahowl::fluid::aero::BladeAero> aero);

    void prestep(double time, double dt) override;
    void poststep(double time, double dt) override;
    void apply_env_model(seahowl::env::EnvModel& env_model, double time) override;
    virtual void build() override;

    /**
     * @brief Applies pitch increment to the blade, rotating it around its longitudinal axis.
     *
     * @param pitch_increment Pitch increment value [rad]
     */
    void apply_pitch_increment(double pitch_increment);

    /**
     * @brief Sets the discretization fractions to use when building the elasto part of the blade.
     *
     * @param[in] fractions Normalized discretization fractions within [0, 1].
     */
    void set_discretization_elasto(const std::vector<double>& fractions);

    /**
     * @brief Sets the discretization fractions to use when building the aero part of the blade.
     *
     * @param[in] fractions Normalized discretization fractions within [0, 1].
     */
    void set_discretization_aero(const std::vector<double>& fractions);

    /**
     * @brief Updates aero positions, rotations, velocities and accelerations from elasto component of the blade.
     */
    void update_positions_aero();

    /**
     * @brief Accumulates aero loads to the elasto component of the blade.
     */
    void update_loads_elasto();

  private:
    /**
     * @brief Initialize blade, called before starting the simulation.
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
