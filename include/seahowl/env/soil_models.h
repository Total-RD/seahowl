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
#include "seahowl/env/model.h"

namespace seahowl {
namespace env {

/**
 * @brief Base class for soil models.
 */
class SoilModel : public Model {
  public:
    /**
     * @brief Returns soil penetration load [N]
     *
     * @param[in] entity Dynamic entity (potentially) penetrating soil.
     * @param[in] contact_area Contact area of entity penetrating soil.
     * @param[in] entity_mass Mass of entity penetrating soil.
     */
    virtual Vector3d get_penetration_load(const EntityDynamic& entity,
                                          double contact_area,
                                          double entity_mass) const = 0;
};

/**
 * @brief Linear soil model with stiffness-based penetration resistance.
 *
 * Implements a simple linear soil model where penetration loads are proportional
 * to penetration depth via normal and shear stiffness coefficients.
 */
class LinearSoilModel : public SoilModel {
  public:
    /** @brief Soil position [m] */
    double soil_position = 0.0;
    /** @brief Soil normal vector. */
    Vector3d soil_normal{0.0, 0.0, 1.0};
    /** @brief Soil normal stiffness. */
    double stiffness_normal = 0.0;
    /** @brief Soil shear stiffness. */
    double stiffness_shear = 0.0;

    /**
     * @brief Constructor.
     */
    LinearSoilModel();

    virtual bool is_inside(const Vector3d& position, double time) const override;

    virtual Vector3d get_penetration_load(const EntityDynamic& entity,
                                          double contact_area,
                                          double entity_mass) const override;
};

}  // namespace env
}  // namespace seahowl
