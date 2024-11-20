#pragma once

#include "tools/test_framework_dataset.h"

#include <gtest/gtest.h>
#include <spdlog/spdlog.h>
#include <filesystem>

using std::filesystem::path;

// The fixture for testing
class FixtureComponents : public ::testing::Test {
  protected:
    FixtureComponents() {
        DATADIR = absolute(path("../data/IEA15MW"));
        spdlog::set_level(spdlog::level::info);
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
        ASSERT_EQ(result, 0);
        ASSERT_FALSE(!error.empty());
    }

    path DATADIR;
    path ref_dir = absolute(path("../tests/unit_tests/data"));
    path test_dir = absolute(path("../tests/unit_tests/data"));
    double rel_error = 1e-2;
    double abs_error = 1e-6;
};
