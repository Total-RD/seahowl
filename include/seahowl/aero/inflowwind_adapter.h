#pragma once

#include <iostream>
#include <cstring>

#include <seahowl/core/turbine.h>
#include <seahowl/elasto/blade_elasto.h>
#include <seahowl/elasto/rotor_elasto.h>
#include <seahowl/elasto/elasto.h>

#include <chrono/core/ChVector.h>
#include <chrono/physics/ChBody.h>
#include <chrono/core/ChMatrix33.h>

#include <seahowl/aero/wind_models.h>

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

namespace aero {

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
    seahowl::aero::InflowWindLib pImpl;

    InflowWindAdapter(std::string InflowInfile, std::string WindWndfile);
    ~InflowWindAdapter();

    void init(double dt);
    chrono::ChVector<double> calcul(double time, chrono::ChVector<double>& position, chrono::ChVector<double>& velocity);
    void end();
    virtual chrono::ChVector<double> get_wind_velocity(chrono::ChVector<double>& position, double time);
};

}  // namespace aero
}  // namespace seahowl