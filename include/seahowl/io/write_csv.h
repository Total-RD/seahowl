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

// Standard library
#include <functional>
#include <map>
#include <string>
#include <vector>

// forward declarations
namespace seahowl {
namespace core {
class System;
class Turbine;
}  // namespace core
}  // namespace seahowl

namespace seahowl {
namespace io {

/**
 * @brief Helper class for outputting custom CSV files.
 */
class CustomCSV {
  public:
    /**
     * @brief Constructor.
     *
     * @param[in] csv_filepath Filepath to CSV file.
     */
    CustomCSV(const std::string& csv_filepath);

    /**
     * @brief Adds function to output variables in CSV rows.
     *
     * @param[in] name Name (header) for the outputted variable(s).
     * @param[in] function Function to call in order to output variables (must return vector of doubles).
     */
    void add_function(const std::string& name, std::function<std::vector<double>()> function);

    /**
     * @brief Adds function to output variables in CSV rows.
     *
     * @param[in] name Name (header) for the outputted variable(s).
     * @param[in] function Function to call in order to output variables (must return a seahowl::Vector3d).
     */
    void add_function(const std::string& name, std::function<seahowl::Vector3d()> function);

    /**
     * @brief Adds function to output variables in CSV rows.
     *
     * @param[in] name Name (header) for the outputted variable(s).
     * @param[in] function Function to call in order to output variables (must return a seahowl::Quaternion).
     */
    void add_function(const std::string& name, std::function<seahowl::Quaternion()> function);

    /**
     * @brief Adds function to output variables in CSV rows.
     *
     * @param[in] name Name (header) for the outputted variable(s).
     * @param[in] function Function to call in order to output variables (must return a double).
     */
    void add_function(const std::string& name, std::function<double()> function);

    /**
     * @brief Outputs CSV row (calls all functions).
     */
    void write_row();

  private:
    /** @brief CSV filepath. */
    std::string csv_filepath = "";
    /** @brief Pairs of <header, function>. */
    std::vector<std::pair<std::string, std::function<std::vector<double>()>>> functions;
    /** @brief Whether the CSV has been initialized or not (outputs headers if false). */
    bool is_initialized = false;
};

}  // namespace io
}  // namespace seahowl
