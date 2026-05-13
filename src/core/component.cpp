// SPDX-License-Identifier: Apache-2.0
// Copyright 2022–2026 TotalEnergies
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//

#include "seahowl/core/component.h"

// Third-party libraries
#include <spdlog/spdlog.h>

// Standard library
#include <typeinfo>

using namespace seahowl::core;

void ComponentDynamic::initialize(double time, double dt) {
    spdlog::debug("Initialization of component: {}.", std::string(typeid(*this).name()));
    if (is_initialized) {
        throw std::runtime_error("Component already initialized: " + std::string(typeid(*this).name()) + ".");
    }

    initialize_this(time, dt);  // component-specific initialization
    is_initialized = true;

    spdlog::debug("Finished initialization of component: {}.", std::string(typeid(*this).name()));
}
