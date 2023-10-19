#include "seahowl/io/read_rotor_perf.h"
#include <seahowl/core/rotor.h>
#include <string>
#include <memory>
#include <vector>
#include <fstream>
#include <spdlog/spdlog.h>
#include <typeinfo>
#include <filesystem>
namespace fs = std::filesystem;
using std::filesystem::path;

void get_disk_perf_from_table(std::string filepath, seahowl::aero::RotorNacelleAssemblyAero& aero) {
    std::string appo;

    if (!fs::exists(filepath)) {
        spdlog::critical("File {} does not exist.", filepath);
        exit(1);
    }

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
            spdlog::error("Disk performance file: empty string at index {}.", ind1 + 1);
            exit(1);
        }
        std::istringstream iss(lineX);
        double number;
        while (iss >> number) {
            pitch.push_back(number);
        }
        Eigen::VectorXd PitchOut = Eigen::Map<Eigen::VectorXd, Eigen::Unaligned>(pitch.data(), pitch.size());

        const std::string& lineX2 = stringVector[ind2 + 1];
        if (lineX2.empty()) {
            spdlog::error("Disk performance file: empty string at index {}.", ind2 + 1);
            exit(1);
        }
        std::istringstream iss2(lineX2);
        while (iss2 >> number) {
            TSR.push_back(number);
        }
        Eigen::VectorXd TSROut = Eigen::Map<Eigen::VectorXd, Eigen::Unaligned>(TSR.data(), TSR.size());

        Eigen::MatrixXd power_coeff;
        power_coeff.resize(TSR.size(), pitch.size());
        power_coeff.setZero();
        for (unsigned ii = 0; ii < TSR.size(); ii++) {
            const std::string& lineX4 = stringVector[ind4 + 2 + ii];
            if (lineX4.empty()) {
                spdlog::error("Disk performance file: empty string at index {}.", ind4 + 2 + ii);
                exit(1);
            }
            std::istringstream iss4(lineX4);
            double numberX;
            int jj = 0;
            while (iss4 >> numberX) {
                power_coeff(ii, jj) = numberX;
                jj += 1;
            }
        }

        // aero.coefficients
        Eigen::MatrixXd thrust_coeff;
        thrust_coeff.resize(TSR.size(), pitch.size());
        thrust_coeff.setZero();
        for (unsigned ii = 0; ii < TSR.size(); ii++) {
            const std::string& lineX5 = stringVector[ind5 + 2 + ii];
            if (lineX5.empty()) {
                spdlog::error("Disk performance: empty string at index {}.", ind5 + 2 + ii);
                exit(1);
            }
            std::istringstream iss5(lineX5);
            double numberX;
            int jj = 0;
            while (iss5 >> numberX) {
                thrust_coeff(ii, jj) = numberX;
                jj += 1;
            }
        }

        aero.disk_coefficients.thrust_coeff = thrust_coeff;
        aero.disk_coefficients.power_coeff = power_coeff;
        aero.disk_coefficients.tsr_list = TSROut;
        aero.disk_coefficients.pitch_list = PitchOut;
    }
}
