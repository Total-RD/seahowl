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

void get_disk_perf_from_table(std::string filepath, seahowl::aero::RotorNacelleAssemblyAero aero) {
    // std::cout<<filepath<<std::endl;

    std::string appo;

    if (!fs::exists(filepath)) {
        throw std::runtime_error("File " + filepath + " does not exist.");
    }

    //	const double PI = 4.0 * atan(1.0);
    std::ifstream myfile(filepath.c_str());

    std::vector<double> pitch;
    std::vector<double> TSR;

    if (myfile.is_open()) {
        std::string line;
        std::istringstream iss1(line);
        unsigned idLine = -1;
        int ind1, ind2, ind3, ind4, ind5 = 0;
        std::vector<std::string> stringVector;
        while (std::getline(myfile, line)) {  // getline (myfile,line);
            idLine += 1;
            stringVector.push_back(line);

            std::size_t check1 = line.find("Pitch");
            std::size_t check2 = line.find("TSR");
            std::size_t check3 = line.find("Wind");
            std::size_t check4 = line.find("Power");
            std::size_t check5 = line.find("Thrust");

            if (check1 != std::string::npos) {
                ind1 = idLine;
            }
            if (check2 != std::string::npos) {
                ind2 = idLine;
            }
            if (check3 != std::string::npos) {
                ind3 = idLine;
            }
            if (check4 != std::string::npos) {
                ind4 = idLine;
            }
            if (check5 != std::string::npos) {
                ind5 = idLine;
            }
        }

        const std::string& lineX = stringVector[ind1 + 1];
        if (lineX.empty()) {
            std::cerr << "Error: Empty string at index " << ind1 + 1 << std::endl;
        }
        std::istringstream iss(lineX);
        double number;
        while (iss >> number) {
            pitch.push_back(number);
        }

        const std::string& lineX2 = stringVector[ind2 + 1];
        if (lineX2.empty()) {
            std::cerr << "Error: Empty string at index " << ind1 + 1 << std::endl;
        }
        std::istringstream iss2(lineX2);
        while (iss2 >> number) {
            TSR.push_back(number);
        }

        // Eigen::MatrixXd power_coeff;
        // power_coeff.resize(pitch.size(),TSR.size());
        // power_coeff.set_Zero();
        // for (unsigned ii=0;ii<10;ii++){
        //     std::istringstream iss(stringVector[ind4+2+ii]);
        //     double number;
        //     int jj = 0;
        //     while (iss >> number) {
        //         power_coeff(ii,jj) = number;
        //     }
        //     jj+=1;
        // }

        // aero.coefficients

        // std::cout<<power_coeff[1][1]<<std::endl;

        // std::cout<<appo<<std::endl;
    }
}
