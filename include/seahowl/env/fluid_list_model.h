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
#include "seahowl/env/fluid_models.h"
#include "seahowl/env/list_model.h"

// Standard library
#include <memory>
#include <vector>

namespace seahowl {
namespace env {
/**
 * @brief Class to store list of fluids.
 */
class FluidListModel : public ListModel<FluidModel> {
  public:
    /**
     * @brief Returns fluid density at given coordinates [kg/m^3]
     *
     * @param[in] position Position at which fluid density is extracted.
     * @param[in] time Time of simulation.
     */
    double get_density(const Vector3d& position, double time) const;

    /**
     * @brief Returns fluid velocity at given coordinates [m/s]
     *
     * @param[in] position Position at which fluid velocity is extracted.
     * @param[in] time Time of simulation.
     */
    Vector3d get_velocity(const Vector3d& position, double time) const;

    /**
     * @brief Returns fluid acceleration at given coordinates [m/s^2]
     *
     * @param[in] position Position at which fluid acceleration is extracted.
     * @param[in] time Time of simulation.
     */
    Vector3d get_acceleration(const Vector3d& position, double time) const;

    /**
     * @brief Sets ramp time for all fluid models.
     * @param[in] start_time Start time of the ramp [s]
     * @param[in] end_time End time of the ramp [s]
     */
    void set_ramp(double start_time, double end_time);
};

}  // namespace env
}  // namespace seahowl
