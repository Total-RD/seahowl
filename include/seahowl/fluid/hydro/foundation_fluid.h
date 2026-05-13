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
#include "seahowl/fluid/component_fluid.h"

namespace seahowl {
namespace fluid {

/** @brief Hydrodynamic module. */
namespace hydro {

class FoundationFluid : public virtual ComponentFluid {
  public:
    /**
     * @brief Virtual destructor.
     */
    virtual ~FoundationFluid() = default;
};

}  // namespace hydro
}  // namespace fluid
}  // namespace seahowl
