#pragma once

#include <seahowl/io/write_csv.h>

#include <functional>
#include <string>
#include <tuple>
#include <vector>
#include <map>

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
};
