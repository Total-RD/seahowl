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
#include "seahowl/core/foundation.h"
#include "seahowl/core/tower.h"
#include "seahowl/elasto/monopile_elasto.h"
#include "seahowl/fluid/hydro/monopile_hydro.h"

namespace seahowl {
namespace core {

/**
 * @brief Monopile of wind turbine.
 */
class Monopile : public Tower, public virtual Foundation {
  public:
    /** @brief Elastodynamic model of the monopile. */
    seahowl::elasto::MonopileElasto& elasto;
    /** @brief Aerodynamic model of the monopile. */
    seahowl::fluid::hydro::MonopileHydro& hydro;

    /**
     * @brief Instantiates tower for communication between elasto and aero components.
     *
     * @param[in] elasto Elastodynamic tower model.
     * @param[in] aero Aerodynamic tower model.
     */
    Monopile(std::shared_ptr<seahowl::elasto::MonopileElasto> elasto,
             std::shared_ptr<seahowl::fluid::hydro::MonopileHydro> hydro)
        : Foundation(elasto, hydro),
          Tower(elasto, hydro),
          ComponentDynamic(elasto, hydro),
          elasto(*elasto),
          hydro(*hydro){};
};

}  // namespace core
}  // namespace seahowl
