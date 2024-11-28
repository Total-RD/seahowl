#include "seahowl/io/write_csv.h"

#include "seahowl/commons/numerics.h"

#include <fstream>
#include <string>
#include <spdlog/spdlog.h>
#include <filesystem>  // C++17

using namespace seahowl::io;
namespace fs = std::filesystem;

CustomCSV::CustomCSV(const std::string& csv_filepath) : csv_filepath(csv_filepath) {}

void CustomCSV::add_function(const std::string& name, std::function<std::vector<double>()> function) {
    functions.push_back(std::pair<std::string, std::function<std::vector<double>()>>(name, function));
}

void CustomCSV::add_function(const std::string& name, std::function<seahowl::Vector3d()> function) {
    functions.push_back(
        std::pair<std::string, std::function<std::vector<double>()>>(name, [function]() -> std::vector<double> {
            auto vec = function();
            return {vec.x(), vec.y(), vec.z()};
        }));
}

void CustomCSV::add_function(const std::string& name, std::function<seahowl::Quaternion()> function) {
    functions.push_back(
        std::pair<std::string, std::function<std::vector<double>()>>(name, [function]() -> std::vector<double> {
            auto quat = function();
            return {quat.w(), quat.x(), quat.y(), quat.z()};
        }));
}

void CustomCSV::add_function(const std::string& name, std::function<double()> function) {
    functions.push_back(std::pair<std::string, std::function<std::vector<double>()>>(
        name, [function]() -> std::vector<double> { return {function()}; }));
}

void CustomCSV::write_row() {
    std::string row_string = "";
    std::string header = "";  // only used if CSV is uninitialized
    if (!is_initialized) {
        // create directories if necessary
        auto fspath = fs::path(csv_filepath);
        auto fspath_parent = fspath.parent_path();
        if (!fspath_parent.empty()) {
            fs::create_directories(fspath_parent);
        }
    }
    for (auto const& function_pair : functions) {
        auto values = function_pair.second();
        size_t ivalue = 0;
        for (auto value : values) {
            row_string.append(std::to_string(value)).append(",");
            if (!is_initialized) {
                if (values.size() == 0) {
                    throw std::runtime_error("CustomCSV: no value outputted for \"" + function_pair.first + "\".");
                }
                if (values.size() == 1) {
                    header.append(function_pair.first).append(",");
                } else {
                    std::string dim = "";
                    if (values.size() == 3) {
                        if (ivalue == 0) {
                            dim = "x";
                        } else if (ivalue == 1) {
                            dim = "y";
                        } else if (ivalue == 2) {
                            dim = "z";
                        }
                    } else {
                        dim = "dim" + std::to_string(ivalue);
                    }
                    size_t pos = function_pair.first.find("(");
                    if (pos != std::string::npos) {
                        pos += header.size();
                        header.append(function_pair.first).insert(pos, dim + " ").append(",");
                    } else {
                        header.append(function_pair.first).append(" " + dim).append(",");
                    }
                }
            }
            ivalue += 1;
        }
    }
    std::ofstream csv_file;
    if (!is_initialized) {
        if (csv_filepath.empty()) {
            throw std::runtime_error("Cannot write to CSV file as no filepath was provided.");
        }
        csv_file.open(csv_filepath);
        csv_file << header << "\n";
        is_initialized = true;
    } else {
        csv_file.open(csv_filepath, std::ios_base::app);
    }
    csv_file << row_string << "\n";
}
