#include "seahowl/io/read_rotor_perf.h"
#include <seahowl/core/rotor.h>
#include <string>
#include <memory>
#include <vector>
#include <fstream>
#include <iostream>
#include <typeinfo>
#include <filesystem>
namespace fs = std::filesystem;
using std::filesystem::path;
using std::filesystem::absolute;

void get_disk_perf_from_table(std::string filepath, seahowl::aero::RotorNacelleAssemblyAero aero) {
    if (!fs::exists(filepath)) {
        throw std::runtime_error("File " + filepath + " does not exist.");
    }
    auto DATADIR = absolute(path(filepath)).parent_path();

    //	const double PI = 4.0 * atan(1.0);
    std::ifstream myfile(filepath.c_str());
    if (myfile.is_open()) {
        unsigned idLine = 0;
        std::string line;
        getline(myfile, line);
        idLine += 1;  // COMMENTS
        getline(myfile, line);
        idLine += 1;
        std::istringstream iss1(line);
    }
}
