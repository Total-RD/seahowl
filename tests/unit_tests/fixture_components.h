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
        spdlog::set_level(spdlog::level::info);

        // set data directory root
        char* const env_datadir = std::getenv("SEAHOWL_DATADIR");
        if (env_datadir == NULL) {
#ifdef SEAHOWL_DATADIR
            DATADIR = path(SEAHOWL_DATADIR);
#else
            spdlog::critical("SEAHOWL_DATADIR not defined (need to set environment variable).");
            std::exit(EXIT_FAILURE);
#endif
        } else {
            DATADIR = path(env_datadir);
        }

        // set test directories roots
        char* const env_testdir = std::getenv("SEAHOWL_TESTDIR");
        if (env_testdir == NULL) {
#ifdef SEAHOWL_TESTDIR
            TESTDIR = path(SEAHOWL_TESTDIR);
#else
            spdlog::critical("SEAHOWL_TESTDIR not defined (need to set environment variable).");
            std::exit(EXIT_FAILURE);
#endif
        } else {
            TESTDIR = path(env_testdir);
        }
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
        ASSERT_EQ(result, 0);
        ASSERT_FALSE(!error.empty());
    }

    path DATADIR;
    path TESTDIR;
    path ref_dir;
    path test_dir;
    double rel_error = 1e-2;
    double abs_error = 1e-6;
};
