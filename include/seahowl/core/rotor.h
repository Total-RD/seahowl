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
#include "seahowl/commons/numerics.h"
#include "seahowl/core/component.h"

// Standard library
#include <memory>
#include <vector>

// forward declarations
namespace seahowl {
namespace core {
class Blade;
}  // namespace core
namespace elasto {
class RotorNacelleAssemblyElasto;
class RotorElasto;
}  // namespace elasto
namespace fluid {
namespace aero {
class RotorNacelleAssemblyAero;
class RotorAero;
}  // namespace aero
}  // namespace fluid
}  // namespace seahowl

namespace seahowl {
namespace core {
/**
 * @brief Rotor.
 */
class Rotor : public ComponentDynamic {
  public:
    /** @brief Elastodynamic model of the Rotor. */
    seahowl::elasto::RotorElasto& elasto;
    /** @brief Aerodynamic model of the Rotor. */
    seahowl::fluid::aero::RotorAero& aero;
    /** @brief Blades of the turbine. */
    std::vector<std::shared_ptr<seahowl::core::Blade>> blades;

    /**
     * @brief Instantiates Rotor for communication between elasto and aero components.
     *
     * @param[in] elasto Elastodynamic Rotor model.
     * @param[in] aero Aerodynamic Rotor model.
     */
    Rotor(std::shared_ptr<seahowl::elasto::RotorElasto> elasto, std::shared_ptr<seahowl::fluid::aero::RotorAero> aero);

    void prestep(double time, double dt) override;
    void poststep(double time, double dt) override;
    void build() override;

  private:
    /**
     * @brief Initialize Rotor, called before starting the simulation.
     *
     * Runs preset and poststep once to make elasto and aero components match.
     *
     * @param[in] time Time of the simulation (usually 0 at init) [s]
     * @param[in] dt Time step length [s]
     */
    void initialize_this(double time, double dt) override;
};

/**
 * @brief Rotor-Nacelle Assembly (RNA) of wind turbine, with both elasto and aero components.
 *
 * This class acts as a "mediator" between the elasto and aero components.
 * The aero position of the RNA is updated using the elasto position.
 */
class RotorNacelleAssembly : public ComponentDynamic {
  public:
    /** @brief Elastodynamic model of the RNA. */
    seahowl::elasto::RotorNacelleAssemblyElasto& elasto;
    /** @brief Aerodynamic model of the RNA. */
    seahowl::fluid::aero::RotorNacelleAssemblyAero& aero;
    /** @brief Rotor. */
    Rotor rotor;

    /**
     * @brief Instantiates rotor for communication between elasto and aero components.
     *
     * @param[in] elasto Elastodynamic RNA model.
     * @param[in] aero Aerodynamic RNA model.
     */
    RotorNacelleAssembly(std::shared_ptr<seahowl::elasto::RotorNacelleAssemblyElasto> elasto,
                         std::shared_ptr<seahowl::fluid::aero::RotorNacelleAssemblyAero> aero);

    void prestep(double time, double dt) override;
    void poststep(double time, double dt) override;
    void apply_env_model(seahowl::env::EnvModel& env_model, double time) override;
    void build() override;

    /**
     * @brief Updates aero positions, rotations, velocities and accelerations from elasto component of the RNA.
     */
    void update_positions_aero();

    /**
     * @brief Returns yaw error [rad]
     *
     * The yaw error is defined as the angle between the rotor disk normal vector to the rotor-disk-averaged relative
     * wind velocity, both projected on global X-Y plane.
     */
    double get_yaw_error() const;

  private:
    /**
     * @brief Initialize RNA, called before starting the simulation.
     *
     * Runs preset and poststep once to make elasto and aero components match.
     *
     * @param[in] time Time of the simulation (usually 0 at init) [s]
     * @param[in] dt Time step length [s]
     */
    void initialize_this(double time, double dt) override;
};

}  // namespace core
}  // namespace seahowl
