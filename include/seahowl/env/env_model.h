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
#include "seahowl/env/fluid_list_model.h"
#include "seahowl/env/soil_list_model.h"

// Standard library
#include <memory>
#include <vector>

// forward declarations
namespace seahowl {
namespace env {
class Model;
}  // namespace env
}  // namespace seahowl

namespace seahowl {
namespace env {
/**
 * @brief Class to store multiple models.
 */
class EnvModel {
  public:
    /** @brief List of fluid models. */
    FluidListModel fluid_models;
    /** @brief List of soil models. */
    SoilListModel soil_models;

    EnvModel() = default;
    ~EnvModel() = default;
    /**
     * @brief Adds a model to the environment.
     *
     * @param[in] model Shared pointer to the model to add (fluid or soil model).
     */
    void add_model(const std::shared_ptr<Model>& model);
};
}  // namespace env
}  // namespace seahowl
