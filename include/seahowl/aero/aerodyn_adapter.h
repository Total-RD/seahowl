#pragma once

#include <iostream>
#include <cstring>

#include <seahowl/core/turbine.h>
#include <seahowl/elasto/blade_elasto.h>
#include <seahowl/elasto/rotor_elasto.h>
#include <seahowl/elasto/entities_elasto.h>
#include <seahowl/commons.h>

/// <summary>
/// Aerodyn module in OpenFAST
/// </summary>
extern "C" {

void AeroDyn_Inflow_C_Init(bool& ADinputFilePassed,
                           const char** ADinputFileString_C,
                           int& ADinputFileStringLength_C,
                           bool& IfWinputFilePassed,
                           const char** IfWinputFileString_C,
                           int& IfWinputFileStringLength_C,
                           char* OutRootName_C,
                           float& gravity_C,
                           float& defFldDens_C,
                           float& defKinVisc_C,
                           float& defSpdSound_C,
                           float& defPatm_C,
                           float& defPvap_C,
                           float& WtrDpth_C,
                           float& MSL2SWL_C,
                           int& InterpOrder_C,
                           double& T_initial_C,
                           double& DT_C,
                           double& TMax_C,
                           bool& storeHHVel,
                           bool& TransposeDCM_in,
                           int& WrVTK_in,
                           int& WrVTK_inType,
                           double& WrVTK_dt,
                           float* VTKNacDim_in,
                           float& VTKHubRad_in,
                           float* HubPos_C,
                           double* HubOri_C,
                           float* NacPos_C,
                           double* NacOri_C,
                           int& NumBlades_C,
                           float* BldRootPos_C,
                           double* BldRootOri_C,
                           int& NumMeshPts_C,
                           float* InitMeshPos_C,
                           double* InitMeshOri_C,
                           int& NumChannels_C,
                           char* OutputChannelNames_C,
                           char* OutputChannelUnits_C,
                           int& ErrStat_C,
                           char* ErrMsg_C);

void AeroDyn_Inflow_C_CalcOutput(double& Time_C,
                                 float* HubPos_C,
                                 double* HubOri_C,
                                 float* HubVel_C,
                                 float* HubAcc_C,
                                 float* NacPos_C,
                                 double* NacOri_C,
                                 float* NacVel_C,
                                 float* NacAcc_C,
                                 float* BldRootPos_C,
                                 double* BldRootOri_C,
                                 float* BldRootVel_C,
                                 float* BldRootAcc_C,
                                 int& NumMeshPts_C,
                                 float* MeshPos_C,
                                 double* MeshOri_C,
                                 float* MeshVel_C,
                                 float* MeshAcc_C,
                                 float* MeshFrc_C,
                                 float* OutputChannelValues_C,
                                 int& ErrStat_C,
                                 char* ErrMsg_C);

void AeroDyn_Inflow_C_UpdateStates(double& Time_C,
                                   double& TimeNext_C,
                                   float* HubPos_C,
                                   double* HubOri_C,
                                   float* HubVel_C,
                                   float* HubAcc_C,
                                   float* NacPos_C,
                                   double* NacOri_C,
                                   float* NacVel_C,
                                   float* NacAcc_C,
                                   float* BldRootPos_C,
                                   double* BldRootOri_C,
                                   float* BldRootVel_C,
                                   float* BldRootAcc_C,
                                   int& NumMeshPts_C,
                                   float* MeshPos_C,
                                   double* MeshOri_C,
                                   float* MeshVel_C,
                                   float* MeshAcc_C,
                                   int& ErrStat_C,
                                   char* ErrMsg_C);

void AeroDyn_Inflow_C_End(int& ErrStat_C, char* ErrMsg_C);
}

namespace seahowl {

namespace core {
class Turbine;
}

namespace aero {

/**@brief Aerodyn_InflowWind wrapping inferface
 *
 */
struct AeroDynInflowLib {
    // Input file handling
    bool ADinputFilePassed = false;   // false: read input info from a primary input file; true: passing info from data
    bool IfWinputFilePassed = false;  // false: read input info from a primary input file; true: passing info from data

    // aerodynamic load computed on mesh point
    float* MeshFrc;

    void SetADINFILE(std::string name);
    void SetIFWINFILE(std::string name);
    void SetOUTNAME(std::string name);

    void SetTime(double time);
    void SetTimeStep(double dt);
    void SetTimeNext(double timenext);
    void SetVTK(int SaveVTK, int VTK_type, double VTK_dt);
    void SetHubPos(float* hubPos);
    void SetHubOri(double* hubOri);
    void SetHubVel(float* hubAcc);
    void SetHubAcc(float* hubAcc);
    void SetNacPos(float* nacPos);
    void SetNacOri(double* nacOri);
    void SetNacVel(float* nacVel);
    void SetNacAcc(float* nacAcc);
    void SetNumBlades(int nBlades);
    void SetBldRootPos(float* bldRootPos);
    void SetBldRootOri(double* bldRootOri);
    void SetBldRootVel(float* bldRootVel);
    void SetBldRootAcc(float* bldRootAcc);
    void SetNumMeshPts(int nMeshPtsAllBlades);
    void SetMeshPos(float* meshPosAllBlades);
    void SetMeshOri(double* meshOriAllBlades);
    void SetMeshVel(float* meshVelAllBlades);
    void SetMeshAcc(float* meshAccAllBlades);
    void SetAeroLoads(float* meshFrcAllBlades);
    void CheckError();

    void Init();
    void Calcul();
    void Update();
    void End();

  private:
    // Input file string
    std::string ADinputFileString;
    std::string IfWinputFileString;

    // Input file string length
    int ADinputFileStringLength;
    int IfWinputFileStringLength;

    /*  OutRootName
     *  If HD writes a file (echo, summary, or other),
     *  use this for the root of the file name.
     */
    char OutRootName[1024];

    // Initial environmental conditions
    // bool MHK = false; //MHK turbine type switch -- disabled for now
    float gravity;      // Gravitational acceleration (m/s^2)
    float defFldDens;   // Air density (kg/m^3)
    float defKinVisc;   // Kinematic viscosity of working fluid (m^2/s)
    float defSpdSound;  // Speed of sound in working fluid (m/s)
    float defPatm;      // Atmospheric pressure (Pa) [used only for an MHK turbine cavitation check]
    float defPvap;      // Vapour pressure of working fluid (Pa) [used only for an MHK turbine cavitation check]
    float WtrDpth;      // Water depth (m)
    float MSL2SWL;      // Offset between still-water level and mean sea level (m) [positive upward]

    // Interpolation order (must be 1: linear, or 2: quadratic)
    int InterpOrder;  // default of linear interpolation

    // Initial time related variables
    double Time;  // initial time
    double DT;    // typical default for AD
    double TMax;  // typical default for AD
    double TimeNext;
    double TimeLast;

    // flags
    bool storeHHVel = false;
    bool TransposeDCM = false;

    // VTK
    int WrVTK;         // default of no vtk output
    int WrVTK_Type;    // default of surface meshes
    double WrVTK_dt;   // vtk save time step
    float* VTKNacDim;  // default nacelle dimension for VTK surface rendering [x0,y0,z0,Lx,Ly,Lz] (m)
    float VTKHubRad;   // default hub radius for VTK surface rendering

    // Initial position of hub and blades
    // used for setup of AD, not used after init.
    float* HubPos;
    double* HubOri;
    float* HubVel;
    float* HubAcc;

    float* NacPos;
    double* NacOri;
    float* NacVel;
    float* NacAcc;

    int NumBlades;
    float* BldRootPos;
    double* BldRootOri;
    float* BldRootVel;
    float* BldRootAcc;

    /* Structural Mesh
     * The number of nodes must be constant throughout simulation. The
     * initial position is given in the initMeshPos array (resize as needed, should be Nx6).
     * Rotations are given in radians assuming small angles.  See note at top of this file.
     */
    int NumMeshPts;  // NumMeshPts = nblade x meshPts of each balde
    float* MeshPos;
    double* MeshOri;
    float* MeshVel;
    float* MeshAcc;

    // number of output channels
    int NumChannels = 0;
    char OutputChannelNames[20 * 8000];
    char OutputChannelUnits[20 * 8000];
    float* OutputChannelValues = new float[100];
    int ErrStat = 0;
    char ErrMsg[1024];
};

class AeroDynAdapter {
  public:
    seahowl::aero::AeroDynInflowLib pImpl;
    std::vector<Vector3d> loads;

    AeroDynAdapter(std::string AerodynInfile, std::string InflowInfile);
    ~AeroDynAdapter();

    void init(double time, double dt, seahowl::core::Turbine& turbine);
    void calcul(double time, seahowl::core::Turbine& turbine);
    void update(double time, double dt, seahowl::core::Turbine& turbine);
    void end(double time, double dt, seahowl::core::Turbine& turbine);
    void update_turbine_variables(seahowl::core::Turbine& turbine);
    void setMotionHub(seahowl::core::Turbine& turbine);
    void setMotionNac(seahowl::core::Turbine& turbine);
    void setMotionRoot(seahowl::core::Turbine& turbine);
    void setMotionMesh(seahowl::core::Turbine& turbine);
};

}  // namespace aero
}  // namespace seahowl
