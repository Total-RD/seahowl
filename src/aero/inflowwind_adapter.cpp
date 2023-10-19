#include "seahowl/aero/inflowwind_adapter.h"

#include <stdexcept>
#include <vector>
#include <string>
#include <iostream>
#include <fstream>
#include <spdlog/spdlog.h>

seahowl::aero::InflowWindAdapter::InflowWindAdapter(std::string InflowInfile, std::string WindWndfile) {
    spdlog::info("Using InflowWind.");
    pImpl.reset(new seahowl::aero::InflowWindLib);
    pImpl->SetIFWINFILE(InflowInfile);
    pImpl->SetWNDINFILE(WindWndfile);
}

seahowl::aero::InflowWindAdapter::~InflowWindAdapter() {}

void seahowl::aero::InflowWindAdapter::init(double dt) {
    pImpl->SetTimeStep(dt);
    pImpl->Init();
}

void seahowl::aero::InflowWindAdapter::end() {
    pImpl->End();
}

seahowl::Vector3d seahowl::aero::InflowWindAdapter::get_wind_velocity(const seahowl::Vector3d& position,
                                                                      double time) const {
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

void seahowl::aero::InflowWindLib::SetIFWINFILE(std::string name) {
    spdlog::info("Set InflowWind INFILE: {name}.", name);
    std::ifstream file(name);
    if (!file.is_open()) {
        spdlog::error("Failed to open inflowwind input file.");
        exit(1);
    }
    std::string line;
    while (std::getline(file, line)) {
        IfWinputFileString += line + '\0';
    }
    IfWinputFileStringLength = IfWinputFileString.length();
    file.close();
}

void seahowl::aero::InflowWindLib::SetWNDINFILE(std::string name) {
    spdlog::info("Set wind.wnd INFILE: {}.", name);
    std::ifstream file(name);
    if (!file.is_open()) {
        spdlog::error("Failed to open wind.wnd input file.");
        exit(1);
    }
    std::string line;
    while (std::getline(file, line)) {
        InputUniformString += line + '\0';
    }
    InputUniformStringLength = InputUniformString.length();
    file.close();
}

void seahowl::aero::InflowWindLib::CheckError() {
    if (ErrStat == 0) {
        return;
    } else if (ErrStat == 1) {
        std::cout << "InflowWind INFO: " << ErrMsg << std::endl;
    } else if (ErrStat == 2) {
        std::cerr << "InflowWind WARNING: " << ErrMsg << std::endl;
    } else {
        std::cerr << "InflowWind ERROR: " << ErrMsg << std::endl;
    }
}

void seahowl::aero::InflowWindLib::SetTimeStep(double dt) {
    DT = dt;
}

void seahowl::aero::InflowWindLib::SetTime(double time) {
    Time = time;
}

void seahowl::aero::InflowWindLib::SetPos(float* Position_C) {
    Position = Position_C;
}

void seahowl::aero::InflowWindLib::SetVel(float* Velocity_C) {
    Velocity = Velocity_C;
}

void seahowl::aero::InflowWindLib::Init() {
    const char* IfWinputFile = IfWinputFileString.c_str();
    const char* IfWUniformFile = InputUniformString.c_str();

    IfW_C_Init(&IfWinputFile, IfWinputFileStringLength, &IfWUniformFile, InputUniformStringLength, NumWindPts, DT,
               NumChannels, OutputChannelNames, OutputChannelUnits, ErrStat, ErrMsg);
    CheckError();
}

void seahowl::aero::InflowWindLib::Calcul() {
    float* Velocity_C = new float[3];
    IfW_C_CalcOutput(Time, Position, Velocity_C, OutputChannelValues, ErrStat, ErrMsg);
    SetVel(Velocity_C);
    CheckError();
}

void seahowl::aero::InflowWindLib::End() {
    IfW_C_End(ErrStat, ErrMsg);
    CheckError();
}
