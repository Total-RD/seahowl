#include "seahowl/env/inflowwind_adapter.h"

#include <stdexcept>
#include <vector>
#include <string>
#include <fstream>
#include <spdlog/spdlog.h>
#include <filesystem>

using namespace seahowl::env;
namespace fs = std::filesystem;

std::vector<std::string> splitString(const std::string& str) {
    std::istringstream iss(str);
    std::vector<std::string> tokens;
    std::string token;
    while (iss >> token) {
        if (!token.empty()) {
            tokens.push_back(token);
        }
    }
    return tokens;
}

InflowWindAdapter::InflowWindAdapter(std::string InflowInfile) {
    spdlog::info("Using InflowWind.");
    pImpl.reset(new InflowWindLib);
    pImpl->SetIFWINFILE(InflowInfile);
    pImpl->SetTimeStep(0.01);  // time step should not matter (not used in InflowWind)
    pImpl->Init();
}

InflowWindAdapter::~InflowWindAdapter() {}

void InflowWindAdapter::end() {
    pImpl->End();
}

seahowl::Vector3d InflowWindAdapter::get_fluid_velocity(const seahowl::Vector3d& position, double time) const {
    float* Pos_C = new float[3];
    for (int i = 0; i < 3; i++) {
        Pos_C[i] = position[i];
    }

    pImpl->SetPos(Pos_C);
    pImpl->SetTime(time);
    pImpl->Calcul();

    auto velocity = seahowl::Vector3d(pImpl->Velocity[0], pImpl->Velocity[1], pImpl->Velocity[2]);
    return velocity;
}

void InflowWindLib::SetIFWINFILE(std::string name) {
    spdlog::info("Set InflowWind INFILE: {}.", name);
    std::ifstream file(name);
    if (!file.is_open()) {
        throw std::runtime_error("Failed to open inflowwind input file.");
    }
    std::string line;
    while (std::getline(file, line)) {
        std::vector<std::string> words = splitString(line);
        if (words.size() > 1) {
            std::string type_lowercase = words[1];
            std::transform(type_lowercase.begin(), type_lowercase.end(), type_lowercase.begin(), ::tolower);

            if (type_lowercase.find("filename") != std::string::npos) {
                size_t insertoffset = 0;
                if (words[0].front() == '"') {
                    insertoffset = 1;
                }
                auto pos = line.find(words[0].front());
                std::string line_before = line;
                auto DATADIR = fs::absolute(fs::path(name)).parent_path();
                line.insert(pos + insertoffset, (DATADIR).string() + "/");
                spdlog::trace("InflowWind: modified path to file to be relative:");
                spdlog::trace("    from: {}", line_before);
                spdlog::trace("    to: {}", line);
            };
        }
        IfWinputFileString += line + '\0';
    }
    IfWinputFileStringLength = IfWinputFileString.length();
    file.close();
}

void InflowWindLib::CheckError() {
    if (ErrStat == 0) {
        return;
    } else if (ErrStat == 1) {
        spdlog::info("InflowWind INFO: {}.", ErrMsg);
    } else if (ErrStat == 2) {
        spdlog::warn("InflowWind WARNING: {}.", ErrMsg);
    } else {
        spdlog::error("InflowWind ERROR: {}.", ErrMsg);
    }
}

void InflowWindLib::SetTimeStep(double dt) {
    DT = dt;
}

void InflowWindLib::SetTime(double time) {
    Time = time;
}

void InflowWindLib::SetPos(float* Position_C) {
    Position = Position_C;
}

void InflowWindLib::SetVel(float* Velocity_C) {
    Velocity = Velocity_C;
}

void InflowWindLib::Init() {
    const char* IfWinputFile = IfWinputFileString.c_str();
    const char* IfWUniformFile = InputUniformString.c_str();

    IfW_C_Init(&IfWinputFile, IfWinputFileStringLength, &IfWUniformFile, InputUniformStringLength, NumWindPts, DT,
               NumChannels, OutputChannelNames, OutputChannelUnits, ErrStat, ErrMsg);
    CheckError();
}

void InflowWindLib::Calcul() {
    float* Velocity_C = new float[3];
    IfW_C_CalcOutput(Time, Position, Velocity_C, OutputChannelValues, ErrStat, ErrMsg);
    SetVel(Velocity_C);
    CheckError();
}

void InflowWindLib::End() {
    IfW_C_End(ErrStat, ErrMsg);
    CheckError();
}
