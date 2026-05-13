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
#include <seahowl/io/write_csv.h>

// Standard library
#include <functional>
#include <map>
#include <string>
#include <tuple>
#include <vector>

class TestFrameworkDataset {
  private:
    bool debug;
    std::string reference_filepath;
    std::string test_filepath;
    //
    std::vector<std::vector<double>> test_data = {};
    std::vector<std::vector<double>> reference_data = {};

  public:
    struct Options {
        bool debug = false;
        std::string reference_filepath;
        std::string test_filepath = "";
    };
    seahowl::io::CustomCSV test_csv;

    TestFrameworkDataset(const Options& options);

    // Compare
    std::tuple<std::string, std::vector<std::vector<double>>> calculate_differences();
    std::tuple<std::string, int> count_errors(const double& abs_error, const double& rel_error);

    void copy_test_to_reference();
};
