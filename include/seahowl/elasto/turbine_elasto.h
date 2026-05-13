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
#include "seahowl/elasto/rotor_elasto.h"
#include "seahowl/elasto/tower_elasto.h"

// Standard library
#include <vector>

// forward declarations
namespace seahowl {
namespace elasto {
class SystemElasto;
class FoundationElasto;
}  // namespace elasto
}  // namespace seahowl

namespace seahowl {
namespace elasto {

/**
 * @brief Wind turbine (blades, rotor-nacelle assembly, tower).
 *
 * This class controls each component, ensuring proper workflow for the elasto part.
 */
class TurbineElasto : public ComponentElasto {
  public:
    // components
    //
    /** @brief Rotor-nacelle assembly of the turbine. */
    std::shared_ptr<seahowl::elasto::RotorNacelleAssemblyElasto> rna;
    /** @brief Tower of the turbine. */
    std::shared_ptr<seahowl::elasto::TowerElasto> tower;
    /** @brief Foundation of the turbine */
    std::shared_ptr<seahowl::elasto::FoundationElasto> foundation;

    /**
     * @brief Constructor.
     *
     * Instantiates rotor component and tower component.
     */
    TurbineElasto();

    virtual void presetup(double fraction) override;
    void build() override;
    virtual void translate(const seahowl::Vector3d& translation_vector) const override;
    virtual void rotate(double angle, const seahowl::Vector3d& axis) const override;
    virtual double get_mass() const override;

  protected:
    virtual void assemble_this(seahowl::elasto::SystemElasto& system) override;
};

}  // namespace elasto
}  // namespace seahowl
