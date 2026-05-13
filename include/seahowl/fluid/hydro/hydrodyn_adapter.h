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

#include "seahowl/commons/numerics.h"
#include "seahowl/elasto/floater_elasto.h"  // TODO: create main body for floater hydro
#include "seahowl/fluid/hydro/floater_hydro.h"
#include "seahowl/fluid/hydro/monopile_hydro.h"

namespace seahowl {

namespace elasto {
class FloaterElasto;
}

namespace fluid {
namespace hydro {

// forward declare (defined in .cpp file)
/**
 * @brief Interface to HydroDyn library.
 */
struct HydroDynLib;

/**
 * @brief Adapter to HydroDyn library.
 */
class HydroDynAdapter {
  public:
    std::unique_ptr<seahowl::hydro::HydroDynLib> interface_hydrodyn;
    std::vector<Vector3d> forces_hydrodyn;
    std::vector<Vector3d> moments_hydrodyn;
    Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic> added_mass_matrix;

    HydroDynAdapter();
    ~HydroDynAdapter();

    void set_hydrodyn_infile(const std::string& hydrodyn_infile);
    void set_seastate_infile(const std::string& seastate_infile);
    void setup_environment(const env::EnvModel& env_model);
    void initialize(double time, double dt, const std::vector<EntityDynamic*>& nodes);
    void compute_loads(double time, const std::vector<EntityDynamic*>& nodes);
    void end();

  private:
    void update_nodes_motion(const std::vector<EntityDynamic*>& nodes);
};

/**
 * @brief Class for floater hydrodynamics with HydroDyn.
 *
 * Only works for floaters with a single hydro body.
 */
class FloaterHydroDyn : public FloaterHydro {
  public:
    FloaterHydroDyn(const std::string& hydrodyn_filepath);

    void set_seastate_infile(const std::string& seastate_infile);
    void setup_environment(const env::EnvModel& env_model) override;
    void compute_env_loads(const env::EnvModel& env_model, double time) override;

    void initialize(double time, double dt) override;

  private:
    /** @brief HydroDyn adapter. */
    std::unique_ptr<seahowl::hydro::HydroDynAdapter> hydrodyn;
};

/**
 * @brief Class for monopile hydrodynamics with HydroDyn.
 */
class MonopileHydroDyn : public MonopileHydro {
  public:
    MonopileHydroDyn(const std::string& hydrodyn_filepath);

    void set_seastate_infile(const std::string& seastate_infile);
    void setup_environment(const env::EnvModel& env_model) override;
    void compute_env_loads(const env::EnvModel& env_model, double time) override;

    void initialize(double time, double dt) override;

  private:
    /** @brief HydroDyn adapter. */
    std::unique_ptr<seahowl::hydro::HydroDynAdapter> hydrodyn;
};

}  // namespace hydro
}  // namespace fluid

}  // namespace seahowl
