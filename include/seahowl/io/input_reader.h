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
#include "seahowl/io/input_structures.h"

namespace seahowl {
namespace io {
/**
 * @brief Interface for reading input data from files.
 */
class InputReader {
  public:
    /**
     * @brief Constructor.
     * @param filepath Path to the input file.
     */
    InputReader(const std::string& filepath_) : filepath(filepath_) {}
    /**
     * @brief Destructor.
     */
    virtual ~InputReader() = default;
    /**
     * @brief Read tower data from a file.
     */
    virtual TowerDb read_tower() = 0;

    /**
     * @brief Read blade data from a file.
     */
    virtual BladeDb read_blade() = 0;

    /**
     * @brief Read RNA data from a file.
     */
    virtual RnaDb read_rna() = 0;

    /**
     * @brief Read environment data from a file.
     */
    virtual EnvironmentDb read_environment() = 0;

    /**
     * @brief Read turbine data from a file.
     */
    virtual TurbineDb read_turbine() = 0;

    /**
     * @brief Read floater data from a file.
     */
    virtual Floaterdb read_floater() = 0;

    /**
     * @brief Read mooring properties data from a file.
     */
    virtual MooringPropertiesDb read_mooring_properties() = 0;

    /**
     * @brief Read main data from a file.
     */
    virtual MainDb read_main() = 0;

  protected:
    std::string filepath;
};

}  // namespace io
}  // namespace seahowl
