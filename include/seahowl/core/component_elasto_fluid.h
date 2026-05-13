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
#include "seahowl/core/component.h"
#include "seahowl/commons/utils.h"

// Standard library
#include <memory>
#include <vector>

// forward declarations
namespace seahowl {
namespace elasto {
class ComponentElasto;
}  // namespace elasto
namespace fluid {
class ComponentFluid;
}  // namespace fluid
}  // namespace seahowl

namespace seahowl {
namespace core {

/**
 * @brief Base class for components coupling elastodynamic and fluid domains.
 *
 * Provides common mapping infrastructure for components that mediate between
 * an elastic structural model and a fluid model (aero or hydro).
 */
class ComponentElastoFluid : public virtual ComponentDynamic {
  protected:
    /** @brief Reference to the elasto component. */
    seahowl::elasto::ComponentElasto& elasto;
    /** @brief Reference to the fluid component. */
    seahowl::fluid::ComponentFluid& fluid;

    /** @brief Mapping of fluid nodes into elasto domain. */
    std::vector<seahowl::DiscretizationPoint> mapping_fluid2elasto_nodes;
    /** @brief Mapping of fluid elements (central point of elements) into elasto domain. */
    std::vector<seahowl::DiscretizationPoint> mapping_fluid2elasto_elements;
    /** @brief Mapping of elasto nodes into fluid domain. */
    std::vector<seahowl::DiscretizationPoint> mapping_elasto2fluid_nodes;

    /**
     * @brief Constructor.
     * @param[in] elasto Shared pointer to the discretized elasto component.
     * @param[in] fluid Shared pointer to the fluid component.
     */
    ComponentElastoFluid(const std::shared_ptr<seahowl::elasto::ComponentElasto> elasto,
                         const std::shared_ptr<seahowl::fluid::ComponentFluid> fluid);

    /**
     * @brief Computes mapping from elasto discretization to fluid discretization.
     */
    void compute_mapping_elasto2fluid();

    /**
     * @brief Computes mapping from fluid to elasto discretization.
     * Element fractions are computed as midpoints between node fractions.
     */
    void compute_mapping_fluid2elasto();
};

}  // namespace core
}  // namespace seahowl
