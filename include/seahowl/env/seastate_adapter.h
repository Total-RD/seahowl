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

#include <iostream>
#include <cstring>
#include <memory>

#include <seahowl/env/wave_models.h>

namespace seahowl {

namespace env {
// forward declare (defined in .cpp file)
/**
 * @brief Interface to SeaState library.
 */
struct SeaStateLib;

/**
 * @brief Adapter to SeaState library.
 */
class SeaStateAdapter : public WaveModel {
  public:
    std::unique_ptr<SeaStateLib> pImpl;

    SeaStateAdapter(std::string seastate_infile);
    ~SeaStateAdapter();

    std::string get_seastate_infile() const { return seastate_infile; }

    double get_water_level(const Vector3d& position, double time) const override;

    // double get_dyn_pressure(const Vector3d& position, double time) const;

  protected:
    double get_density_this(const Vector3d& position, double time) const override;
    virtual Vector3d get_velocity_this(const Vector3d& position, double time) const override;
    virtual Vector3d get_acceleration_this(const Vector3d& position, double time) const override;
    std::string seastate_infile = "";
};

}  // namespace env
}  // namespace seahowl
