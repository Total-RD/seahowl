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
#include "seahowl/elasto/component_elasto.h"
#include "seahowl/elasto/reference_point_elasto.h"

namespace seahowl {
namespace elasto {

/**
 * @brief Tower of wind turbine as an elastodynamic FEA component.
 *
 * Towers are discretized into beam elements that are either simple Timoshenko elements described through lineic
 * density, foreaft and sideside stiffnesses.
 */
class TowerElasto : public ComponentElastoFEA {
  public:
    /** @brief List of reference points describing the tower properties along its longitudinal axis. */
    std::vector<TowerReferencePointElasto> reference_points;
    /** @brief List of discretized points (interpolated reference points) describing the tower properties. */
    std::vector<TowerReferencePointElasto> discretized_points;
    /** @brief Height of the tower (absolute value above ground / sea water level) [m] */
    double height = 0.0;
    /** @brief Height of the base of the tower (absolute value above ground / sea water level) [m] */
    double base_height = 0.0;

    /**
     * @brief Constructor.
     */
    TowerElasto();

    void build() override;

    /**
     * @brief Returns tower base moment (first node of first element of tower) [Nm]
     */
    Vector3d get_tower_base_moment() const;

    /**
     * @brief Returns tower top moment (second node of last element of tower) [Nm]
     */
    Vector3d get_tower_top_moment() const;

    /**
     * @brief Returns tower base force (first node of first element of tower) [N]
     */
    Vector3d get_tower_base_force() const;

    /**
     * @brief Returns tower top force (second node of last element of tower) [N]
     */
    Vector3d get_tower_top_force() const;

    virtual void reset_loads() override;

  private:
    /**
     * @brief Builds the tower with Timoshenko elements (lineic density, foreaft stiffness, sideside stiffness).
     */
    void build_elements_tapered_timoshenko();
};

}  // namespace elasto
}  // namespace seahowl
