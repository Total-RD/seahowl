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
#include "seahowl/core/system.h"
#include "seahowl/elasto/system_elasto.h"
#include "seahowl/fluid/system_fluid.h"
#include "seahowl/io/config_manager.h"
#include "seahowl/io/output_manager.h"

// Standard library
#include <memory>
#include <string>

namespace seahowl {
namespace core {

class Simulation {
  public:
    std::unique_ptr<System> system_core;
    std::unique_ptr<seahowl::io::OutputManager> outputs;

    double dt = 0.025;
    double duration = 1000.0;
    bool is_initialized = false;

    Simulation();

    // Enable move semantics
    Simulation(Simulation&&) = default;
    Simulation& operator=(Simulation&&) = default;

    // Disable copy
    Simulation(const Simulation&) = delete;
    Simulation& operator=(const Simulation&) = delete;

    void populate_from_file(const std::string& filepath);
    void populate_from_config();
    void initialize_from_config();
    void initialize();
    void step();
    void run_all();
    seahowl::io::app::ConfigManager& getConfigManager();

  private:
    std::shared_ptr<seahowl::elasto::SystemElasto> system_elasto;
    std::shared_ptr<seahowl::fluid::SystemFluid> system_fluid;
    int nstep = 0;
    double t_output_next = 0.0;
    std::string main_filepath;
    seahowl::io::app::ConfigManager config;
};

}  // namespace core
}  // namespace seahowl
