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
#include "seahowl/fluid/aero/tower_aero.h"
#include "seahowl/fluid/hydro/foundation_fluid.h"

namespace seahowl {
namespace fluid {
namespace hydro {

/**
 * @brief Monopile of wind turbine as an hydrodynamic component.
 */
class MonopileHydro : public seahowl::aero::TowerAero, public virtual FoundationFluid {};

}  // namespace hydro
}  // namespace fluid
namespace hydro = fluid::hydro;
}  // namespace seahowl
