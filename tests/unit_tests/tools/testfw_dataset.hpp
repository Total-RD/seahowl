#pragma once

#include <functional>
#include <string>
#include <tuple>
#include <vector>

class TestFwDataSet {
  private:
    bool debug;
    std::string reference_filepath;
    std::string test_filepath;
    std::vector<std::function<std::vector<double>()>> test_functions;
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
        std::vector<std::function<std::vector<double>()>> test_functions = {};
    };

    TestFwDataSet(const Options& options)
        : debug(options.debug),
          reference_filepath(options.reference_filepath),
          test_filepath(options.test_filepath),
          dimensions(options.dimensions),
          test_functions(options.test_functions) {}

    // Fill test data
    void testDataSet(const std::vector<std::vector<double>>& data);
    void testAddRow(const std::vector<double>& data);
    void testAdd();

    // Compare
    std::tuple<std::string, std::vector<std::vector<double>>> differencesCalculate();
    std::tuple<std::string, int> differencesErrCount(const double& abs_error, const double& rel_error);
};
