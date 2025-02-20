#pragma once

#include <iostream>
#include <cstring>
#include <memory>

#include <seahowl/env/wind_models.h>

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

    // Input file string length
    int IfWinputFileStringLength;

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
    /** @brief Level below which returned velocity is (0.0, 0.0, 0.0), used for z<0 when using TurbSim for example. */
    double zmin = 0.0;

    std::unique_ptr<InflowWindLib> pImpl;

    InflowWindAdapter(std::string InflowInfile);
    ~InflowWindAdapter();

    void end();

  protected:
    virtual Vector3d get_fluid_velocity_this(const Vector3d& position, double time) const override;
    virtual Vector3d get_fluid_acceleration_this(const Vector3d& position, double time) const override;
};

}  // namespace env
}  // namespace seahowl
