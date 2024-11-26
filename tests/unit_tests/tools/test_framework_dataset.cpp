#include "test_framework_dataset.h"

#include <filesystem>
#include <iostream>
#include <string>
#include <tuple>
#include <vector>
#include <cmath>
#include <spdlog/spdlog.h>
#include <sstream>

#include "csv.hpp"

void print_data(const vector<vector<double>>& data) {
    ostringstream oss;
    for (const auto& row : data) {
        for (const auto& val : row) {
            oss << std::setprecision(10) << val << " ";
        }
        oss << std::endl;
    }
    spdlog::debug(oss.str());
}

TestFrameworkDataset::TestFrameworkDataset(const Options& options)
    : debug(options.debug),
      reference_filepath(options.reference_filepath),
      test_filepath(options.test_filepath),
      test_csv(options.test_filepath) {}

tuple<string, vector<vector<double>>> TestFrameworkDataset::calculate_differences() {
    string error = "";
    vector<vector<double>> result;

    // Read reference file
    test_data = csv_read(test_filepath, true);
    reference_data = csv_read(reference_filepath, true);

    // Debug
    if (debug) {
        spdlog::debug("Reference data:");
        print_data(reference_data);
        spdlog::debug("Test data:");
        print_data(test_data);
    }

    // Compare dimensions
    if (test_data.size() != reference_data.size()) {
        error = "Error: Dimension mismatch between test data and reference data.";
        return make_tuple(error, result);
    }

    for (size_t i = 0; i < test_data.size(); ++i) {
        if (test_data[i].size() != reference_data[i].size()) {
            error = "Error: Dimension mismatch between test data and reference data on line " + to_string(i);
            return make_tuple(error, result);
        }
        vector<double> row = {};
        for (size_t j = 0; j < test_data[i].size(); ++j) {
            row.push_back(test_data[i][j] - reference_data[i][j]);
        }
        result.push_back(row);
    }

    if (debug) {
        spdlog::debug("Differences:");
        print_data(result);
    }

    return make_tuple(error, result);
}

tuple<string, int> TestFrameworkDataset::count_errors(const double& rel_error, const double& abs_error) {
    string error = "";
    int result = 0;

    // calculate differences
    vector<vector<double>> differences;
    tie(error, differences) = calculate_differences();
    if (!error.empty()) {
        return make_tuple(error, result);
    }

    // check
    for (size_t i = 0; i < differences.size(); ++i) {
        for (size_t j = 0; j < differences[i].size(); ++j) {
            if (abs(reference_data[i][j]) > 1.e-12) {
                if (abs(differences[i][j]) / abs(reference_data[i][j]) > rel_error) {
                    result++;
                }
            } else {
                if (abs(differences[i][j]) > abs_error) {
                    result++;
                }
            }
        }
    }

    return make_tuple(error, result);
}
