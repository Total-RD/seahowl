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

// Local test headers
#include "tools/get_env_var.h"
#include "tools/test_framework_dataset.h"

// Third-party libraries
#include <gtest/gtest.h>
#include <spdlog/spdlog.h>

// Standard library
#include <filesystem>

using std::filesystem::path;

// The fixture for testing
class FixtureComponents : public ::testing::Test {
  protected:
    FixtureComponents() {
        spdlog::set_level(spdlog::level::info);

        // set data directory root
        DATADIR = get_data_dir();
        // set test directories roots
        TESTDIR = get_test_dir();

        root_dir = path(TESTDIR) / "unit_tests/data";
        ref_dir = path(TESTDIR) / "unit_tests/data";
        test_dir = path(TESTDIR) / "unit_tests/data";
    }

    ~FixtureComponents() override {
        // You can do clean-up work that doesn't throw exceptions here.
    }

    // If the constructor and destructor are not enough for setting up
    // and cleaning up each test, you can define the following methods:

    void SetUp() override {
        // Code here will be called immediately after the constructor (right
        // before each test).
    }

    void TearDown() override {
        // Code here will be called immediately after each test (right
        // before the destructor).
    }

    void EvaluateTest(TestFrameworkDataset& test_dataset) {
        auto [error, result] = test_dataset.count_errors(rel_error, abs_error);
        if (error.empty()) {
            spdlog::info(" Tests Report : nb errors = " + std::to_string(result));
        } else {
            spdlog::error("Tests Report ERROR : " + error);
        }
        if (dump_test_to_reference) {
            test_dataset.copy_test_to_reference();
        }
        ASSERT_EQ(result, 0);
        ASSERT_FALSE(!error.empty());
    }

    path DATADIR;
    path TESTDIR;
    path root_dir;
    path ref_dir;
    path test_dir;
    double rel_error = 1e-3;
    double abs_error = 1e-6;
    bool dump_test_to_reference = false;
};
