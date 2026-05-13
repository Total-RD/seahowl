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
 * @brief Base class for wind models.
 */
class WindModel : public FluidModel {
  public:
    /** @brief Air density [kg/m^3] */
    double density = 1.225;
    /** @brief Direction of gravitational acceleration. */
    Vector3d direction_gravity{0.0, 0.0, -1.0};

    virtual bool is_inside(const Vector3d& position, double time = 0.0) const override;
    virtual double get_density_this(const Vector3d& position, double time) const override;
};

/** @brief Sheared wind model. This model and derived models assume a ground level at z=0.0. */
class ShearedWind : public WindModel {
  public:
    /** @brief Wind shear coefficient. */
    double shear_coefficient = 0.0;
    /** @brief Reference height (where constant velocity is defined) [m] */
    double reference_height = 150.0;
};

/** @brief Constant wind models. */
class ConstantWind : public ShearedWind {
  public:
    /** @brief Wind velocity [m/s] */
    Vector3d wind_velocity;

    /**
     * @brief Constructor.
     */
    ConstantWind();

    /**
     * @brief Sets wind velocity.
     *
     * @param[in] wind_velocity Wind velocity to use as constant [m/s]
     */
    void set_wind_velocity(Vector3d velocity);

  protected:
    virtual Vector3d get_velocity_this(const Vector3d& position, double time) const override;
    virtual Vector3d get_acceleration_this(const Vector3d& position, double time) const override;
};

/** @brief Wind ramp model. */
class WindRamp : public ShearedWind {
  public:
    /** @brief Starting time of ramp [s] */
    double time_start = 0.0;
    /** @brief Ending time of ramp [s] */
    double time_end = 0.0;
    /** @brief Wind velocity at starting of ramp [m/s] */
    Vector3d wind_velocity_start{0.0, 0.0, 0.0};
    /** @brief Wind velocity at end of ramp [m/s] */
    Vector3d wind_velocity_end{0.0, 0.0, 0.0};

    /**
     * @brief Constructor.
     */
    WindRamp();

    /**
     * @brief Sets wind ramp.
     *
     * @param[in] velocity_start Wind velocity at starting of ramp [m/s]
     * @param[in] time_start Time at which the ramp starts [s]
     * @param[in] velocity_end Wind velocity at end of ramp [m/s]
     * @param[in] time_end Time at which the ramp ends [s]
     */
    void set_wind_ramp(const Vector3d& velocity_start,
                       double time_start,
                       const Vector3d& velocity_end,
                       double time_end);

  protected:
    virtual Vector3d get_velocity_this(const Vector3d& position, double time) const override;
    virtual Vector3d get_acceleration_this(const Vector3d& position, double time) const override;
};

}  // namespace env
}  // namespace seahowl
