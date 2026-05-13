// SPDX-License-Identifier: Apache-2.0
// Copyright 2022–2026 TotalEnergies
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//

#include "seahowl/io/viz_insitu.h"

// SEAHOWL headers
#include "seahowl/core/system.h"

// Third-party libraries
#include <spdlog/spdlog.h>

using namespace seahowl::io;

VisualizationInSitu::VisualizationInSitu() {
    spdlog::info("No concrete in situ visualization application selected.");
}

void VisualizationInSitu::initialize_elasto(seahowl::elasto::SystemElasto& system) {}

void VisualizationInSitu::initialize(seahowl::core::System& system) {}

void VisualizationInSitu::draw() {}
