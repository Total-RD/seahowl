// SPDX-License-Identifier: Apache-2.0
// Copyright 2022–2026 TotalEnergies
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//

#include "seahowl/env/inflowwind_adapter.h"

// Third-party libraries
#include <spdlog/spdlog.h>

// Standard library
#include <filesystem>
#include <fstream>
#include <stdexcept>
#include <string>
#include <vector>

using namespace seahowl::env;
namespace fs = std::filesystem;

/// <summary>
/// Inflowwind module in OpenFAST
/// </summary>
extern "C" {

void IfW_C_Init(int& IfWinputFilePassed_C,
                const char** InputFileString_C,
                int& InputFileStringLength_C,
                char* OutRootName_C,
                int& NumWindPts_C,
                double& DT_C,
                int& DebugLevel_C,
                int& NumChannels_C,
                char* OutputChannelNames_C,
                char* OutputChannelUnits_C,
                int& ErrStat_C,
                char* ErrMsg_C);

void IfW_C_CalcOutput(double& Time_C,
                      float* Positions_C,
                      float* Velocities_C,
                      float* OutputChannelValues_C,
                      int& ErrStat_C,
                      char* ErrMsg_C);

void IfW_C_End(int& ErrStat_C, char* ErrMsg_C);

void IfW_C_GetFlowFieldPointer(void** FlowFieldPtr, int& ErrStat_C, char* ErrMsg_C);

void IfW_C_SetFlowFieldPointer(void* FlowFieldPtr, int& ErrStat_C, char* ErrMsg_C);
}

/**
 * @brief InflowWind wrapping inferface
 */
struct seahowl::env::InflowWindLib {
    ~InflowWindLib();

    void SetIFWINFILE(std::string name);
    void CheckError();

    void SetTimeStep(double dt);

    void Init();
    void Calcul(double time, float* position, float* velocity);
    void End();
    void* GetFlowFieldPointer();
    void SetFlowFieldPointer(void* FlowFieldPtr);

  private:
    // Input file string
    std::string IfWinputFileString;

    // Input file string length
    int IfWinputFileStringLength = 0;

    int IfWinputFilePassed = 0;

    // Number of wind points
    int NumWindPts = 1;

    // Time step
    double DT = 0.0;

    // Debug level
    int DebugLevel = 0;

    // number of output channels
    int NumChannels = 0;
    char OutputChannelNames[20 * 8000];
    char OutputChannelUnits[20 * 8000];
    float* OutputChannelValues = new float[100];
    int ErrStat = 0;
    char ErrMsg[1024];
};

InflowWindLib::~InflowWindLib() {
    delete[] OutputChannelValues;
    End();
}

void InflowWindLib::SetIFWINFILE(std::string name) {
    spdlog::info("Set InflowWind INFILE: {}.", name);
    IfWinputFileString = name;
    IfWinputFileStringLength = IfWinputFileString.length();
}

void InflowWindLib::CheckError() {
    if (ErrStat == 0) {
        return;
    } else if (ErrStat == 1) {
        spdlog::info("InflowWind INFO: \"{}\".", ErrMsg);
    } else if (ErrStat == 2) {
        spdlog::warn("InflowWind WARNING: \"{}\".", ErrMsg);
    } else {
        throw std::runtime_error("InflowWind ERROR: \"" + std::string(ErrMsg) + "\".");
    }
}

void InflowWindLib::SetTimeStep(double dt) {
    DT = dt;
}

void InflowWindLib::Init() {
    const char* IfWinputFile = IfWinputFileString.c_str();
    char OutRootName[1024];
    strcpy(OutRootName, "IfW");

    IfW_C_Init(IfWinputFilePassed, &IfWinputFile, IfWinputFileStringLength, OutRootName, NumWindPts, DT, DebugLevel,
               NumChannels, OutputChannelNames, OutputChannelUnits, ErrStat, ErrMsg);
    CheckError();
}

void InflowWindLib::Calcul(double time, float* position, float* velocity) {
    IfW_C_CalcOutput(time, position, velocity, OutputChannelValues, ErrStat, ErrMsg);
    CheckError();
}

void InflowWindLib::End() {
    IfW_C_End(ErrStat, ErrMsg);
    CheckError();
}

void* InflowWindLib::GetFlowFieldPointer() {
    void* FlowFieldPtr;
    IfW_C_GetFlowFieldPointer(&FlowFieldPtr, ErrStat, ErrMsg);
    CheckError();
    return FlowFieldPtr;
}

void InflowWindLib::SetFlowFieldPointer(void* FlowFieldPtr) {
    IfW_C_SetFlowFieldPointer(&FlowFieldPtr, ErrStat, ErrMsg);
    CheckError();
}

InflowWindAdapter::InflowWindAdapter(const std::string& inflowwind_infile_) : inflowwind_infile(inflowwind_infile_) {
    spdlog::info("Using InflowWind.");
    pImpl.reset(new InflowWindLib);
    pImpl->SetIFWINFILE(inflowwind_infile);
    pImpl->SetTimeStep(0.01);  // time step should not matter (not used in InflowWind)
    pImpl->Init();
}

InflowWindAdapter::~InflowWindAdapter() {}

void InflowWindAdapter::end() {
    pImpl->End();
}

bool InflowWindAdapter::is_inside(const Vector3d& position, double time) const {
    if (position.z() < zmin) {
        return false;
    }
    return true;
}

seahowl::Vector3d InflowWindAdapter::get_velocity_this(const seahowl::Vector3d& position, double time) const {
    float* Pos_C = new float[3];
    for (int i = 0; i < 3; i++) {
        Pos_C[i] = position[i];
    }
    float* Vel_C = new float[3];
    pImpl->Calcul(time, Pos_C, Vel_C);

    auto velocity = seahowl::Vector3d(Vel_C[0], Vel_C[1], Vel_C[2]);

    delete[] Pos_C;
    delete[] Vel_C;

    return velocity;
}

seahowl::Vector3d InflowWindAdapter::get_acceleration_this(const seahowl::Vector3d& position, double time) const {
    return Vector3d(0.0, 0.0, 0.0);
}
