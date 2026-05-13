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
#include "seahowl/io/input_reader.h"

namespace seahowl {
namespace io {

/**
 * @brief Interface for reading input data from files.
 */
class InputReaderJson : public InputReader {
  public:
    /**
     * @brief Constructor that accepts a file path.
     * @param filepath Path to the JSON file.
     */
    explicit InputReaderJson(const std::string& filepath) : InputReader(filepath) {}

    /**
     * @brief Read tower data from a file.
     */
    TowerDb read_tower();

    /**
     * @brief Read blade data from a file.
     */
    BladeDb read_blade();

    /**
     * @brief Read RNA data from a file.
     */
    RnaDb read_rna();

    /**
     * @brief Read environment data from a file.
     */
    EnvironmentDb read_environment();

    /**
     * @brief Read turbine data from a file.
     */
    TurbineDb read_turbine();

    /**
     * @brief Read floater data from a file.
     */
    Floaterdb read_floater();

    /**
     * @brief Read mooring properties data from a file.
     */
    MooringPropertiesDb read_mooring_properties();

    /**
     * @brief Read main data from a file.
     */
    MainDb read_main();
};

}  // namespace io
}  // namespace seahowl
