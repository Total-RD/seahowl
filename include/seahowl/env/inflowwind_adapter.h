#pragma once

#include <iostream>
#include <cstring>
#include <memory>

#include <seahowl/env/wind_models.h>

/// <summary>
/// Inflowwind module in OpenFAST
/// </summary>
extern "C" {

void IfW_C_Init(const char** InputFileString_C,
                int& InputFileStringLength_C,
                const char** InputUniformString_C,
                int& InputUniformStringLength_C,
                int& NumWindPts_C,
                double& DT_C,
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
}

namespace seahowl {

namespace core {
class Turbine;
}

namespace env {

/**@brief InflowWind wrapping inferface
 *
 */
struct InflowWindLib {
    // velocity
    float* Velocity;

    void SetIFWINFILE(std::string name);
    void SetWNDINFILE(std::string name);
    void CheckError();

    void SetTimeStep(double dt);
    void SetTime(double time);
    void SetPos(float* Position_C);
    void SetVel(float* Velocity_C);

    void Init();
    void Calcul();
    void End();

  private:
    // Input file string
    std::string IfWinputFileString;
    std::string InputUniformString;

    // Input file string length
    int IfWinputFileStringLength;
    int InputUniformStringLength;

    // Number of wind points
    int NumWindPts = 1;

    // Time step
    double Time;
    double DT;

    // positions
    float* Position;

    // number of output channels
    int NumChannels = 0;
    char OutputChannelNames[20 * 8000];
    char OutputChannelUnits[20 * 8000];
    float* OutputChannelValues = new float[100];
    int ErrStat = 0;
    char ErrMsg[1024];
};

class InflowWindAdapter : public WindModel {
  public:
    std::unique_ptr<InflowWindLib> pImpl;

    InflowWindAdapter(std::string InflowInfile);
    ~InflowWindAdapter();

    void end();
    virtual Vector3d get_fluid_velocity(const Vector3d& position, double time) const override;
};

}  // namespace env
}  // namespace seahowl
