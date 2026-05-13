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
#include "seahowl/env/wind_models.h"

// Standard library
#include <cstring>
#include <iostream>
#include <memory>

namespace seahowl {

namespace core {
class Turbine;
}

namespace env {
// forward declare (defined in .cpp file)
/**
 * @brief Interface to InflowWind library.
 */
struct InflowWindLib;

/**
 * @brief Adapter to InflowWind library.
 */
class InflowWindAdapter : public WindModel {
  public:
    /** @brief Level below which returned velocity is (0.0, 0.0, 0.0), used for z<0 when using TurbSim for example. */
    double zmin = 0.0;

    std::unique_ptr<InflowWindLib> pImpl;

    virtual bool is_inside(const Vector3d& position, double time = 0.0) const override;

    InflowWindAdapter(const std::string& inflowwind_infile);
    ~InflowWindAdapter();

    std::string get_inflowwind_infile() const { return inflowwind_infile; }

    void end();

  protected:
    virtual Vector3d get_velocity_this(const Vector3d& position, double time) const override;
    virtual Vector3d get_acceleration_this(const Vector3d& position, double time) const override;
    std::string inflowwind_infile = "";
};

}  // namespace env
}  // namespace seahowl
