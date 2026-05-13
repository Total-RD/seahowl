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

// Standard library
#include <memory>

// forward declarations
namespace seahowl {
namespace core {
class System;
}  // namespace core
namespace elasto {
class SystemElasto;
}  // namespace elasto
}  // namespace seahowl

namespace seahowl {
namespace io {
class VisualizationInSitu {
  public:
    VisualizationInSitu();

    /**
     * @brief Initialization of in situ visualization for system.
     *
     * @param[in] system System to visualize.
     */
    virtual void initialize(seahowl::core::System& system);

    /**
     * @brief Initialization of in situ visualization for system.
     *
     * @param[in] system System to visualize.
     */
    virtual void initialize_elasto(seahowl::elasto::SystemElasto& system_elasto);

    /**
     * @brief Draw elements of systems (rigid bodies, FEA beams, etc).
     */
    virtual void draw();
};
}  // namespace io
}  // namespace seahowl
