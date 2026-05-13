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
#include "seahowl/fluid/hydro/foundation_fluid.h"

// Standard library
#include <memory>

// forward declarations
namespace seahowl {
namespace fluid {
namespace hydro {
class MooringSystemHydro;
}  // namespace hydro
}  // namespace fluid
namespace hydro = fluid::hydro;
namespace env {
class FluidModel;
}  // namespace env
}  // namespace seahowl

namespace seahowl {
namespace fluid {

namespace hydro {

/**
 * @brief Floater of wind turbine as an hydrodynamic component.
 */
class FloaterHydro : public FoundationFluid {
  public:
    /** @brief Mooring system of the floater. */
    std::shared_ptr<MooringSystemHydro> mooring_system;
    /** @brief Main body of floater.*/
    std::unique_ptr<seahowl::EntityDynamicEigen> body_main;

    /**
     * @brief Constructor.
     */
    FloaterHydro();

    void build() override;

    void compute_env_loads(const env::EnvModel& fluid_model, double time) override;

    Vector3d get_force_hydro();

    Vector3d get_torque_hydro();

    Eigen::Matrix<double, 6, 6> get_added_mass_matrix();

  protected:
    /** @brief External force acting on floater [N] */
    Vector3d force_hydro = {0.0, 0.0, 0.0};

    /** @brief External torque acting on floater [Nm] */
    Vector3d torque_hydro = {0.0, 0.0, 0.0};

    /** @brief Added mass matrix of floater. */
    Eigen::Matrix<double, 6, 6> added_mass_matrix = Eigen::Matrix<double, 6, 6>::Zero();
};

}  // namespace hydro
}  // namespace fluid
}  // namespace seahowl
