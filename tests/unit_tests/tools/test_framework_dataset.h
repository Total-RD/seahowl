#pragma once

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
    std::map<std::string, std::function<std::vector<double>()>> test_functions_map;
    std::vector<std::string> test_functions_names = {};
    //
    std::vector<std::string> dimensions = {};
    //
    std::vector<std::vector<double>> test_data = {};
    std::vector<std::vector<double>> reference_data = {};

  public:
    struct Options {
        bool debug = false;
        std::string reference_filepath;
        std::string test_filepath = "";
        std::vector<std::string> dimensions = {};
        std::map<std::string, std::function<std::vector<double>()>> test_functions;
    };

    TestFrameworkDataset(const Options& options)
        : debug(options.debug),
          reference_filepath(options.reference_filepath),
          test_filepath(options.test_filepath),
          dimensions(options.dimensions),
          test_functions_map(options.test_functions) {}

    // Fill test data
    void set_dataset(const std::vector<std::vector<double>>& data);
    void add_test_function(const std::string& name, std::function<std::vector<double>()> test_function);
    void add_test_function(const std::string& name, std::function<double()> test_function);
    void add_row_data(const std::vector<double>& data);
    void add_row();

    // Compare
    std::tuple<std::string, std::vector<std::vector<double>>> calculate_differences();
    std::tuple<std::string, int> count_errors(const double& abs_error, const double& rel_error);
};
