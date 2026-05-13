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
#include "seahowl/env/fluid_models.h"

namespace seahowl {
namespace env {

/**
 * @brief Base class for wave models.
 */
class WaveModel : public FluidModel {
  public:
    /** @brief Water density [kg/m^3] */
    double density = 1025;
    /** @brief Mean water level [m] */
    double mean_water_level = 0.0;
    /** @brief Free surface normal. */
    Vector3d surface_normal{0.0, 0.0, 1.0};
    /** @brief Water depth [m] */
    double water_depth = 0.0;

    virtual bool is_inside(const Vector3d& position, double time) const override;

    /**
     * @brief Returns water level (free surface elevation) at given position and time.
     *
     * @param[in] position Horizontal position at which water level is computed.
     * @param[in] time Time of simulation.
     * @return Water level (free surface elevation) [m]
     */
    virtual double get_water_level(const Vector3d& position, double time) const = 0;
};

/**
 * @brief Model for still water (no wave, no current).
 */
class StillWater : public WaveModel {
  public:
    /**
     * @brief Constructor.
     */
    StillWater();

    virtual double get_water_level(const Vector3d& position, double time) const override;

  protected:
    virtual double get_density_this(const Vector3d& position, double time) const override;
    virtual Vector3d get_velocity_this(const Vector3d& position, double time) const override;
    virtual Vector3d get_acceleration_this(const Vector3d& position, double time) const override;
};

/**
 * @brief Model for constant current with only horizontal velocity components.
 */
class CurrentConstant : public StillWater {
  public:
    /** @brief Horizontal velocity of fluid at the free surface [m/s] */
    double velocity_surface = 0.0;
    /** @brief Horizontal velocity of fluid at the seabed [m/s] */
    double velocity_seabed = 0.0;
    /** @brief Current direction. */
    Vector3d direction{1.0, 0.0, 0.0};
    /** @brief Power factor for for power law exponent (1/power_factor). */
    double power_factor = 7;

    /**
     * @brief Constructor.
     */
    CurrentConstant();

  protected:
    virtual Vector3d get_velocity_this(const Vector3d& position, double time) const override;
    virtual Vector3d get_acceleration_this(const Vector3d& position, double time) const override;
};

}  // namespace env
}  // namespace seahowl
