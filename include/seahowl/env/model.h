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
#include "seahowl/commons/numerics.h"

namespace seahowl {
/** @brief Environmental models (wind, wave, soil). */
namespace env {

/**
 * @brief Base class for models.
 */
class Model {
  public:
    /**
     * @brief Returns true is the placement of the model (false otherwise).
     *
     * @param[in] position Position to assess whether inside model or not.
     * @param[in] time Time of simulation [s]
     */
    virtual bool is_inside(const Vector3d& position, double time = 0.0) const = 0;
    /**
     * @brief Virtual destructor.
     */
    virtual ~Model() = default;
};

}  // namespace env
}  // namespace seahowl
