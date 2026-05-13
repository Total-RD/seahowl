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
#include "seahowl/commons/numerics.h"
#include "seahowl/fluid/component_fluid.h"

// Standard library
#include <memory>

// forward declarations
namespace seahowl {
namespace fluid {
namespace aero {
class BladeAero;
class TowerAero;
}  // namespace aero
}  // namespace fluid
namespace aero = fluid::aero;
namespace env {
class EnvModel;
}  // namespace env
}  // namespace seahowl

namespace seahowl {
namespace fluid {
namespace aero {

/** @brief Base class for rotor aerodynamic models. */
class RotorAero : public ComponentFluid {
  public:
    /** @brief List of blades. */
    std::vector<std::shared_ptr<seahowl::aero::BladeAero>> blades;
    /** @brief Hub. */
    EntityDynamicEigen body_hub;
    /** @brief Radius of hub [m] */
    double hub_radius = 0.0;
    /** @brief Aerodynamic torque on hub [Nm] */
    double hub_torque_aero = 0.0;
    /** @brief Aerodynamic thrust on hub [N] */
    double hub_thrust_aero = 0.0;
    /** @brief Total radius of the rotor (hub + blades) [m] */
    double radius = 0.0;
    /** @brief Azimuth of rotor [rad] */
    double azimuth = 0.0;
    /** @brief Collective pitch of blades [rad] */
    double pitch_collective = 0.0;
    /** @brief Disk averaged wind speed [m/s] */
    Vector3d disk_averaged_wind_velocity{0.0, 0.0, 0.0};

    /**
     * @brief Initializes the rotor aerodynamic model.
     */
    virtual void initialize() = 0;
    virtual void compute_env_loads(const env::EnvModel& env_model, double time) = 0;

    /**
     * @brief Computes disk-averaged wind velocity.
     *
     * The disk-averaged wind velocity is computed by averaging the wind velocity along all blade.
     */
    virtual void compute_disk_averaged_wind_velocity(const env::EnvModel& fluid_model, double time);
};

/** @brief Rotor aerodynamic model using Blade Element Theory (BET). */
class RotorAeroBET : public RotorAero {
  public:
    RotorAeroBET();

    virtual void build() override;
    virtual void initialize() override;
    virtual void compute_env_loads(const env::EnvModel& env_model, double time) override;

    /**
     * @brief Computes radius, distances from tip and hub, and chord solidity on all aero nodes of blades.
     */
    void compute_radii_distances_solidity();
};

/** @brief Rotor aerodynamic model using Blade Element Momentum Theory (BEMT). */
class RotorAeroBEMT : public RotorAeroBET {
  public:
    /** @brief Reference to tower aero. */
    TowerAero& tower_ref;
    /** Whether to take tip loss into account or not. */
    bool has_tip_loss = true;
    /** Whether to take hub loss into account or not. */
    bool has_hub_loss = true;
    /** Whether to take tower shadow into account or not. */
    bool has_tower_shadow = true;

    RotorAeroBEMT(TowerAero& tower_ref);

    virtual void build() override;
    virtual void initialize() override;
    virtual void compute_env_loads(const env::EnvModel& env_model, double time) override;
};

/** @brief Tabulated aerodynamic coefficients for the actuator disk rotor model. */
struct DiskCoefficients {
    /** @brief Thrust coefficient table indexed by pitch and TSR. */
    Eigen::MatrixXd thrust_coeff;
    /** @brief Power coefficient table indexed by pitch and TSR. */
    Eigen::MatrixXd power_coeff;
    /** @brief List of tip-speed ratio values for table interpolation. */
    Eigen::VectorXd tsr_list;
    /** @brief List of pitch angle values for table interpolation [deg]. */
    Eigen::VectorXd pitch_list;

    /** @brief Interpolates thrust and power coefficients from tables for a given TSR and pitch. */
    seahowl::Vector2d get_disk_coefficients_from_table(double TSR, double pitch);
};

/** @brief Rotor aerodynamic model using the actuator disk approach. */
class RotorAeroDisk : public RotorAero {
  public:
    /** @brief The tables of actuator disk coefficients. */
    DiskCoefficients disk_coefficients;

    virtual void build() override{};
    void initialize() override;
    void compute_env_loads(const env::EnvModel& env_model, double time) override;
};

/**
 * @brief Rotor-Nacelle Assembly (RNA) of wind turbine as an aero component.
 */
class RotorNacelleAssemblyAero : public ComponentFluid {
  public:
    /** @brief Rotor. */
    std::shared_ptr<RotorAero> rotor;
    /** @brief Nacelle. */
    EntityDynamicEigen body_nacelle;

    /**
     * @brief Constructor.
     */
    RotorNacelleAssemblyAero();

    void compute_env_loads(const env::EnvModel& env_model, double time) override;

    /**
     * @brief Builds rotor.
     */
    void build() override;

    /**
     * @brief Initializes rotor related variables with current configuration.
     */
    void initialize();
};

}  // namespace aero
}  // namespace fluid
}  // namespace seahowl
