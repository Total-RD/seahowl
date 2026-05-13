// SPDX-License-Identifier: Apache-2.0
// Copyright 2022–2026 TotalEnergies
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//

#include "seahowl/env/seastate_adapter.h"

#include <stdexcept>
#include <vector>
#include <string>
#include <fstream>
#include <spdlog/spdlog.h>
#include <filesystem>

using namespace seahowl::env;
namespace fs = std::filesystem;

/**
 * @brief SeaState module in OpenFAST
 * Long term this should be moved to a header file in OpenFAST
 */
extern "C" {
//
#define CHANNEL_NAME_SIZE 20       // maximum number of characters in a channel name (without c_null termination)
#define PASSED_STRING_LENGTH 1025  // file name length for fixed length filenames (includes c_null termination)
#define ERROR_MSG_LEN 8197         // total character count in error message (includes c_null termination)
#define MAX_OUT_CHANS 20000        // maximum number of channels in any OF c-bind interface

void SeaSt_C_PreInit(
    float& Gravity_C,                        // in  - gravity (use negative to point downwards) (m/s^2)
    float& WtrDens_C,                        // in  - water density (kg/m^3)
    float& WtrDpth_C,                        // in  - water depth (MSL to sea-floor) (m)
    float& MSL2SWL_C,                        // in  - Mean sea level to still water level (m)
    int& DebugLevel_In,                      // in  - for debugging interface (0: none, 4: everything including meshes)
    char OutVTKDir_C[PASSED_STRING_LENGTH],  // in  - Directory to put all vtk output.  Length InfStrLen
    int& WrVTK_in,                           // in  - Write VTK outputs [0: none, 1: init only, 2: animation]
    double& WrVTK_inDT,                      // in  - Timestep between VTK writes
    int& ErrStat_C,                          // out - Error status (0: none, 1: Info, 2: warn, 3: severe, 4: fatal)
    char* ErrMsg_C);                         // out - Message returned about error (empty if none)

void SeaSt_C_Init(char InputFile_C[PASSED_STRING_LENGTH],    // in  - SeaState input file (absolute or relative path)
                  char OutRootName_C[PASSED_STRING_LENGTH],  // in  - rootname for output summary, output, or echo files
                                                             // (absolute or relative path)
                  double& TimeInterval_C,                    // in  - timestep (s)
                  double& TimeMax_C,                         // in  - maximum size (s)
                  double& WaveTimeShift_C,                   // timeshift (s)
                  int& NumChannels_C,                        // out - number of output channels
                  char* OutputChannelNames_C,  // out - channel names - each channel name is CHANNEL_NAME_SIZE in length
                  char* OutputChannelUnits_C,  // out - channel units - each channel unit is CHANNEL_NAME_SIZE in length
                  int& ErrStat_C,              // out - Error status (0: none, 1: Info, 2: warn, 3: severe, 4: fatal)
                  char* ErrMsg_C);             // out - Message returned about error (empty if none)

// NOTE: The only reason to call CalcOutput is for visualization of the sea surface
void SeaSt_C_CalcOutput(double& Time_C,                // in  - current time (s)
                        float* OutputChannelValues_C,  // out - output channel values
                        int& ErrStat_C,   // out - Error status (0: none, 1: Info, 2: warn, 3: severe, 4: fatal)
                        char* ErrMsg_C);  // out - Message returned about error (empty if none)

void SeaSt_C_End(int& ErrStat_C,   // out - Error status (0: none, 1: Info, 2: warn, 3: severe, 4: fatal)
                 char* ErrMsg_C);  // out - Message returned about error (empty if none)

void SeaSt_C_GetWaveFieldPointer(void** WaveFieldPtr,  // out - pointer to wavefield data (fotran pointer converted to C
                                                       // pointer.  Store as int* locally)
                                 int& ErrStat_C,  // out - Error status (0: none, 1: Info, 2: warn, 3: severe, 4: fatal)
                                 char* ErrMsg_C);  // out - Message returned about error (empty if none)

void SeaSt_C_SetWaveFieldPointer(void* WaveFieldPtr,  // in  - pointer to wavefield data - retrieved from a
                                                      // GetWaveFieldPointer call. Stored as int* locally.
                                 int& ErrStat_C,  // out - Error status (0: none, 1: Info, 2: warn, 3: severe, 4: fatal)
                                 char* ErrMsg_C);  // out - Message returned about error (empty if none)

// Get the fluid velocity, acceleration, and node-in-water status at time+position coordinate
// NOTE: if wave stretching is turned off, the SWL is used as the cutoff for the nodeInWater and for Vel / Acc values
// NOTE: Velocity and Accel could be split, but use same routine on Fortran backend
void SeaSt_C_GetFluidVelAcc(double& Time_C,      // in  - current time (s)
                            float* Pos_c,        // in  - position in 3D (m). Relative to SWL
                            float* Vel_c,        // out - velocity at requested point.  (m/s)
                            float* Acc_c,        // out - acceleration at requested point.  (m/s^2)
                            int& NodeInWater_C,  // out - node is in or out of water (0: out of water, 1: in water)
                            int& ErrStat_C,      // out - Error status (0: none, 1: Info, 2: warn, 3: severe, 4: fatal)
                            char* ErrMsg_C);     // out - Message returned about error (empty if none)

void SeaSt_C_GetSurfElev(double& Time_C,   // in  - current time (s)
                         float* Pos_c,     // in  - position in 2D (m).
                         float& Elev_C,    // out - wave elevation relative to SWL (m)
                         int& ErrStat_C,   // out - Error status (0: none, 1: Info, 2: warn, 3: severe, 4: fatal)
                         char* ErrMsg_C);  // out - Message returned about error (empty if none)

// the following exists in the fortran library, but is not implemented in the adapter yet
void SeaSt_C_GetSurfNorm(double& Time_C,    // in  - current time (s)
                         float* Pos_c,      // in  - position in 2D (m)
                         float& NormVec_C,  // out - unit vector normal to surface (-)
                         int& ErrStat_C,    // out - Error status (0: none, 1: Info, 2: warn, 3: severe, 4: fatal)
                         char* ErrMsg_C);   // out - Message returned about error (empty if none)

/* TODO: missing in OpenFAST 4.2.0
void SeaSt_C_GetFluidDynP(double& Time_C,   // in  - current time (s)
                         float* Pos_c,     // in  - position in 2D (m).
                         float& DynP_C,    // out - dynamic pressure at requested location (N/m^2)
                         int& ErrStat_C,   // out - Error status (0: none, 1: Info, 2: warn, 3: severe, 4: fatal)
                         char* ErrMsg_C);  // out - Message returned about error (empty if none)
*/

// NOTE: this routine overestimates the range when 2nd order is used
void SeaSt_C_GetElevMinMaxEstimate(float& min,       // out - minimum wave elevation across entire wavefield (m)
                                   float& max,       // out - maximum wave elevation across entire wavefield (m)
                                   int& ErrStat_C,   // out
                                   char* ErrMsg_C);  // out

double SeaSt_C_GetDens(float& Density,   // out - density (kg/m^3) - constant throughout simulation
                       int& ErrStat_C,   // out
                       char* ErrMsg_C);  // out

double SeaSt_C_GetDpth(float& Depth,     // out - water depth (m) - constant throughout simulation
                       int& ErrStat_C,   // out
                       char* ErrMsg_C);  // out

double SeaSt_C_GetMSL2SWL(float& Density,   // out - Distance MSL to SWL (m) - constant throughout simulation
                          int& ErrStat_C,   // out
                          char* ErrMsg_C);  // out
}

/**
 * @brief SeaState wrapping inferface
 */
struct seahowl::env::SeaStateLib {
    ~SeaStateLib();

    void SetSSINFILE(std::string name);
    void CheckError();

    void SetTimeStep(double dt);
    void SetTMax(double TMax);

    void* GetWaveFieldPointer();
    void SetWaveFieldPointer(void* WaveFieldPtr);

    void Init();
    void Calcul(double time);
    double GetWaterLevel(const seahowl::Vector3d& position, double time);
    /* TODO: missing in OpenFAST 4.2.0
        double GetDynPressure(const seahowl::Vector3d& position, double time); */
    double GetFluidDensity();
    double GetWaterDepth();
    double GetWaterMSL2SWL();
    seahowl::Vector3d GetFluidVelocity(const seahowl::Vector3d& position, double time);
    seahowl::Vector3d GetFluidAcceleration(const seahowl::Vector3d& position, double time);
    void End();

  private:
    // Time step
    double DT = 0.25;     // (s) -- expected dt for calling
    double TMax = 600;    // (s) -- maximum time for simulation
    double TShift = 0.0;  // (s) -- for phase shifting

    // Env vars (check if seastate checks these values)
    float Gravity = 9.80665;  // (m/s^2)
    float WtrDens = 1025;     // (kg/m^3)
    float WtrDpth = 200;      // (m)
    float MSL2SWL = 0;        // Offset between still-water level and mean sea level (m) [positive upward]

    // Debug level
    int DebugLevel = 0;  // FIXME: change to 0

    // VTK
    int WrVTK = 0;
    double WrVTK_DT = 0.25;

    // Input file string
    std::string SSinputFileString;

    // number of output channels
    int NumChannels = 0;
    char OutputChannelNames[20 * 8000];
    char OutputChannelUnits[20 * 8000];
    float* OutputChannelValues = new float[100];
    int ErrStat = 0;
    char ErrMsg[ERROR_MSG_LEN - 1];

    // Water level
    float min_water_level = 0;  // Set during init
    float max_water_level = 0;  // Set during init
};

SeaStateLib::~SeaStateLib() {
    End();
}

void SeaStateLib::SetSSINFILE(std::string name) {
    spdlog::info("Set SeaState INFILE: {}.", name);
    SSinputFileString = name;
}

void SeaStateLib::CheckError() {
    if (ErrStat == 0) {
        return;
    } else if (ErrStat == 1) {
        spdlog::info("SeaState INFO: \"{}\".", ErrMsg);
    } else if (ErrStat == 2) {
        spdlog::warn("SeaState WARNING: \"{}\".", ErrMsg);
    } else {
        throw std::runtime_error("SeaState ERROR: \"" + std::string(ErrMsg) + "\".");
    }
}

void SeaStateLib::SetTimeStep(double dt) {
    DT = dt;
}

void SeaStateLib::SetTMax(double tmax) {
    TMax = tmax;
}

void SeaStateLib::Init() {
    char SSinputFile[PASSED_STRING_LENGTH - 1];
    strcpy(SSinputFile, SSinputFileString.c_str());
    char OutRootName[PASSED_STRING_LENGTH - 1];
    strcpy(OutRootName, "SS");
    char OutVTKDir[PASSED_STRING_LENGTH - 1];
    strcpy(OutVTKDir, "./vtk/");  // FIXME: need some way to set this

    SeaSt_C_PreInit(Gravity, WtrDens, WtrDpth, MSL2SWL, DebugLevel, OutVTKDir, WrVTK, WrVTK_DT, ErrStat, ErrMsg);
    CheckError();

    SeaSt_C_Init(SSinputFile, OutRootName, DT, TMax, TShift, NumChannels, OutputChannelNames, OutputChannelUnits,
                 ErrStat, ErrMsg);
    CheckError();

    // get min and max wave elevations to streamline node in water checks where possible
    SeaSt_C_GetElevMinMaxEstimate(min_water_level, max_water_level, ErrStat, ErrMsg);
    CheckError();
}

double SeaStateLib::GetWaterLevel(const Vector3d& position, double time) {
    float* Pos_C = new float[2];
    for (int i = 0; i < 2; i++) {
        Pos_C[i] = position[i];
    }
    int ErrStat = 0;
    float Elev_C = 0;
    char ErrMsg[ERROR_MSG_LEN - 1];
    SeaSt_C_GetSurfElev(time,     // in  - current time (s)
                        Pos_C,    // in  - position in 2D (m).
                        Elev_C,   // out - wave elevation relative to SWL (m)
                        ErrStat,  // out - Error status (0: none, 1: Info, 2: warn, 3: severe, 4: fatal)
                        ErrMsg);  // out - Message returned about error (empty if none)
    CheckError();

    delete[] Pos_C;

    return GetWaterMSL2SWL() + Elev_C;
};

/* TODO: missing in OpenFAST 4.2.0
double SeaStateLib::GetDynPressure(const Vector3d& position, double time) {
    spdlog::debug("Node position ({}, {}, {})", position.x(), position.y(), position.z());
    float* Pos_C = new float[3];
    float DynP_C = 0;
    for (int i = 0; i < 3; i++) {
        Pos_C[i] = position[i];
    }
    int ErrStat = 0;
    float Elev_C = 0;
    char ErrMsg[ERROR_MSG_LEN - 1];
    SeaSt_C_GetFluidDynP(time,     // in  - current time (s)
                         Pos_C,    // in  - position in 2D (m).
                         DynP_C,   // out - Dynamic pressure (N/m^2)
                         ErrStat,  // out - Error status (0: none, 1: Info, 2: warn, 3: severe, 4: fatal)
                         ErrMsg);  // out - Message returned about error (empty if none)
    CheckError();

    delete[] Pos_C;

    spdlog::debug("Dynamic pressure: {}", DynP_C);
    return DynP_C;
};
*/

double SeaStateLib::GetFluidDensity() {
    float Density = 0;
    SeaSt_C_GetDens(Density, ErrStat, ErrMsg);
    CheckError();
    return Density;
}

double SeaStateLib::GetWaterDepth() {
    float Depth = 0;
    SeaSt_C_GetDpth(Depth, ErrStat, ErrMsg);
    CheckError();
    return Depth;
}

double SeaStateLib::GetWaterMSL2SWL() {
    float MSL2SWL = 0;
    SeaSt_C_GetMSL2SWL(MSL2SWL, ErrStat, ErrMsg);
    CheckError();
    return MSL2SWL;
}

seahowl::Vector3d SeaStateLib::GetFluidVelocity(const seahowl::Vector3d& position, double time) {
    float* Pos_C = new float[3];
    for (int i = 0; i < 3; i++) {
        Pos_C[i] = position[i];
    }
    Pos_C[2] -= GetWaterMSL2SWL();
    float* Vel_C = new float[3];
    float* Acc_C = new float[3];
    int NodeInWater_C = 1;

    SeaSt_C_GetFluidVelAcc(time,           // in  - current time (s)
                           Pos_C,          // in  - position in 3D (m). Relative to SWL
                           Vel_C,          // out - velocity at requested point.  (m/s)
                           Acc_C,          // out - acceleration at requested point.  (m/s^2)
                           NodeInWater_C,  // out - node is in or out of water (0: out of water, 1: in water)
                           ErrStat,        // out - Error status (0: none, 1: Info, 2: warn, 3: severe, 4: fatal)
                           ErrMsg);        // out - Message returned about error (empty if none)
    CheckError();

    auto velocity = seahowl::Vector3d(Vel_C[0], Vel_C[1], Vel_C[2]);

    delete[] Pos_C;
    delete[] Vel_C;
    delete[] Acc_C;

    return velocity;
}

seahowl::Vector3d SeaStateLib::GetFluidAcceleration(const seahowl::Vector3d& position, double time) {
    float* Pos_C = new float[3];
    for (int i = 0; i < 3; i++) {
        Pos_C[i] = position[i];
    }
    Pos_C[2] -= GetWaterMSL2SWL();
    float* Vel_C = new float[3];
    float* Acc_C = new float[3];
    int NodeInWater_C = 1;
    int ErrStat = 0;
    char ErrMsg[ERROR_MSG_LEN - 1];
    SeaSt_C_GetFluidVelAcc(time,           // in  - current time (s)
                           Pos_C,          // in  - position in 3D (m). Relative to SWL
                           Vel_C,          // out - velocity at requested point.  (m/s)
                           Acc_C,          // out - acceleration at requested point.  (m/s^2)
                           NodeInWater_C,  // out - node is in or out of water (0: out of water, 1: in water)
                           ErrStat,        // out - Error status (0: none, 1: Info, 2: warn, 3: severe, 4: fatal)
                           ErrMsg);        // out - Message returned about error (empty if none)
    CheckError();

    auto acceleration = seahowl::Vector3d(Acc_C[0], Acc_C[1], Acc_C[2]);

    delete[] Pos_C;
    delete[] Vel_C;
    delete[] Acc_C;

    return acceleration;
}

// NOTE: this only needs to be used if VTK outputs are requested.
void SeaStateLib::Calcul(double time) {
    SeaSt_C_CalcOutput(time, OutputChannelValues, ErrStat, ErrMsg);
    CheckError();
}

void SeaStateLib::End() {
    SeaSt_C_End(ErrStat, ErrMsg);
    CheckError();
}

void* SeaStateLib::GetWaveFieldPointer() {
    void* WaveFieldPtr;
    SeaSt_C_GetWaveFieldPointer(&WaveFieldPtr, ErrStat, ErrMsg);
    CheckError();
    return WaveFieldPtr;
}

void SeaStateLib::SetWaveFieldPointer(void* WaveFieldPtr) {
    SeaSt_C_SetWaveFieldPointer(&WaveFieldPtr, ErrStat, ErrMsg);
    CheckError();

    SeaSt_C_GetElevMinMaxEstimate(min_water_level, max_water_level, ErrStat, ErrMsg);
    CheckError();
}

// FIXME: add way to set size and number of timesteps
SeaStateAdapter::SeaStateAdapter(std::string seastate_infile) : seastate_infile(seastate_infile) {
    spdlog::info("Using SeaState.");
    pImpl.reset(new SeaStateLib);
    pImpl->SetSSINFILE(seastate_infile);
    pImpl->SetTimeStep(0.25);  // With number of timesteps, sets the total wave simlulation time. 0.25 typical
    pImpl->SetTMax(600);       // 600 second simulation
    pImpl->Init();
}

SeaStateAdapter::~SeaStateAdapter() {}

double SeaStateAdapter::get_water_level(const Vector3d& position, double time) const {
    return pImpl->GetWaterLevel(position, time);
};

/* TODO: missing in OpenFAST 4.2.0
double SeaStateAdapter::get_dyn_pressure(const Vector3d& position, double time) const {
    return pImpl->GetDynPressure(position, time);
};
*/

double SeaStateAdapter::get_density_this(const seahowl::Vector3d& position, double time) const {
    return pImpl->GetFluidDensity();
}

seahowl::Vector3d SeaStateAdapter::get_velocity_this(const seahowl::Vector3d& position, double time) const {
    return pImpl->GetFluidVelocity(position, time);
}

seahowl::Vector3d SeaStateAdapter::get_acceleration_this(const seahowl::Vector3d& position, double time) const {
    return pImpl->GetFluidAcceleration(position, time);
}

// double SeaStateAdapter::get_max_water_level() const {
//     return max_water_level;
// }
