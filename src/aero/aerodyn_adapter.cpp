#include <seahowl/aero/aerodyn_adapter.h>

#include <seahowl/aero/turbine_aero.h>
#include <seahowl/aero/blade_aero.h>
#include <seahowl/env/fluid_models.h>

#include <stdexcept>
#include <vector>
#include <string>
#include <iostream>
#include <fstream>
#include <spdlog/spdlog.h>
// #include <numeric>
// #include <sstream>

extern "C" {

void ADI_C_PreInit(int& NumTurbines_C,
                   bool& TransposeDCM_in,
                   int& PointLoadOutput_in,
                   int& DebugLevel_in,
                   int& ErrStat_C,
                   char* ErrMsg_C);

void ADI_C_SetupRotor(int& iWT_c,
                      bool& TurbineIsHAWT_c,
                      float* TurbOrigin_C,
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
                      int* MeshPtToBladeNum_C,
                      int& ErrStat_C,
                      char* ErrMsg_C);

void ADI_C_SetRotorMotion(int& iWT_c,
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

/*
void seahowl::aero::AeroDynInflowLib::GetRotorLoads() {
void ADI_C_GetRotorLoads()
}

void seahowl::aero::AeroDynInflowLib::GetDiskAvgVel() {
void ADI_C_GetDiskAvgVel()
}
*/

void ADI_C_Init(bool& ADinputFilePassed,
                const char** ADinputFileString_C,
                int& ADinputFileStringLength_C,
                bool& IfWinputFilePassed,
                const char** IfWinputFileString_C,
                int& IfWinputFileStringLength_C,
                char* OutRootName_C,
                char* OutVTKDir_C,
                float& gravity_C,
                float& defFldDens_C,
                float& defKinVisc_C,
                float& defSpdSound_C,
                float& defPatm_C,
                float& defPvap_C,
                float& WtrDpth_C,
                float& MSL2SWL_C,
                int& InterpOrder_C,
                double& DT_C,
                double& TMax_C,
                bool& storeHHVel,
                int& WrVTK_in,
                int& WrVTK_inType,
                double& WrVTK_dt,
                float* VTKNacDim_in,
                float& VTKHubRad_in,
                int& wrOuts_C,
                double& DT_Outs_C,
                int& NumChannels_C,
                char* OutputChannelNames_C,
                char* OutputChannelUnits_C,
                int& ErrStat_C,
                char* ErrMsg_C);

void ADI_C_CalcOutput(double& Time_C, float* OutputChannelValues_C, int& ErrStat_C, char* ErrMsg_C);

void ADI_C_UpdateStates(double& Time_C, double& TimeNext_C, int& ErrStat_C, char* ErrMsg_C);

void ADI_C_End(int& ErrStat_C, char* ErrMsg_C);
}

/**@brief Aerodyn_InflowWind wrapping inferface
 *
 */
struct seahowl::aero::AeroDynInflowLib {
    // Input file handling
    bool ADinputFilePassed = false;   // false: read input info from a primary input file; true: passing info from data
    bool IfWinputFilePassed = false;  // false: read input info from a primary input file; true: passing info from data

    void set_aerodyn_infile(const std::string& name);
    void set_inflowwind_infile(const std::string& name);
    void set_outfile_name(const std::string& name);

    void initialize_arrays(int NumBlades, int NumMeshPts);
    void set_time(double time);

    void CheckError();
    void Init();
    void Calcul();
    void Update();
    void End();

  public:
    // aerodynamic load computed on mesh point
    float* MeshFrc;

    // Input file string
    std::string ADinputFileString;
    std::string IfWinputFileString;

    // Input file string length
    int ADinputFileStringLength;
    int IfWinputFileStringLength;

    bool TurbineIsHAWT = true;
    int NumTurbines = 1;
    int iWT = 1;
    int PointLoadOutput_in = 1;
    int DebugLevel_in = 1;
    float* TurbOrigin;
    int* MeshPtToBladeNum;

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

    // Aero calculation method -- AeroProjMod
    // APM_BEM_NoSweepPitchTwist - 1 -  "Original AeroDyn model where momentum balance is done in the
    // WithoutSweepPitchTwist system" APM_BEM_Polar             - 2 -  "Use staggered polar grid for momentum balance in
    // each annulus" APM_LiftingLine           - 3 -  "Use the blade lifting line (i.e. the structural) orientation
    // (currently for OLAF with VAWT)" Type of aerodynamic projection
    int AeroProjMod;

    // Interpolation order (must be 1: linear, or 2: quadratic)
    int InterpOrder;  // default of linear interpolation

    // Initial time related variables
    double Time;  // initial time
    double DT;    // typical default for AD
    double TMax;  // typical default for AD
    double TimeLast;

    // flags
    bool storeHHVel = false;
    bool TransposeDCM = false;

    // VTK
    int WrVTK = 0;       // default of no vtk output
    int WrVTK_Type = 1;  // default of surface meshes
    double WrVTK_dt;     // vtk save time step
    float* VTKNacDim;    // default nacelle dimension for VTK surface rendering [x0,y0,z0,Lx,Ly,Lz] (m)
    float VTKHubRad;     // default hub radius for VTK surface rendering

    // Write outputs to file
    int wrOuts;      // write ADI output file
    double DT_Outs;  // timestep to write output file from ADI

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

seahowl::aero::AeroDynAdapter::AeroDynAdapter() {
    spdlog::debug("Initialising Aerodyn15 adapter");
    pImpl = std::make_unique<AeroDynInflowLib>();
    pImpl->ADinputFilePassed = false;
    pImpl->IfWinputFilePassed = false;
    pImpl->set_outfile_name("Turbine");
}

seahowl::aero::AeroDynAdapter::~AeroDynAdapter() {}

void seahowl::aero::AeroDynAdapter::set_infiles(const std::string& AerodynInfile, const std::string& InflowInfile) {
    pImpl->set_aerodyn_infile(AerodynInfile);
    pImpl->set_inflowwind_infile(InflowInfile);
}

void seahowl::aero::AeroDynAdapter::initialize(double time, double dt, seahowl::aero::TurbineAero& turbine) {
    pImpl->DT = dt;
    pImpl->set_time(time);

    // initialize arrays of interface
    int nblades = turbine.rna.rotor->blades.size();
    int npoints = 0;
    for (auto& blade : turbine.rna.rotor->blades) {
        npoints += blade->nodes.size();
    }
    pImpl->initialize_arrays(nblades, npoints);

    // associate points to blade idx
    int idx_blade = 0;
    for (auto& blade : turbine.rna.rotor->blades) {
        int idx_node = 0;
        for (auto& node : blade->nodes) {
            pImpl->MeshPtToBladeNum[idx_node] = idx_blade + 1;
            idx_node += 1;
        }
        idx_blade += 1;
    }

    // update turbine variables
    update_turbine_variables(turbine);

    pImpl->Init();
}

void seahowl::aero::AeroDynAdapter::compute_loads(double time, seahowl::aero::TurbineAero& turbine) {
    pImpl->set_time(time);
    update_turbine_variables(turbine);
    if (time > 0.) {
        pImpl->Update();
        pImpl->Calcul();
    } else {
        pImpl->Calcul();
    }
}

void seahowl::aero::AeroDynAdapter::end() {
    pImpl->End();
}

void seahowl::aero::AeroDynAdapter::update_turbine_variables(seahowl::aero::TurbineAero& turbine) {
    update_hub_motion(turbine);
    update_nacelle_motion(turbine);
    update_roots_motion(turbine);
    update_mesh_motion(turbine);
}

void seahowl::aero::AeroDynAdapter::update_hub_motion(seahowl::aero::TurbineAero& turbine) {
    // Get the information about hub
    auto& hub = turbine.rna.rotor->body_hub;
    auto hubPos = hub.get_position();
    auto hubOri = hub.get_rotation().toRotationMatrix();  // get a rotation matrix 3x3
    auto hubTranVel = hub.get_velocity();
    auto hubRotVel = hub.get_rotational_velocity(false);  // in global frame
    auto hubTranAcc = hub.get_acceleration();
    auto hubRotAcc = hub.get_rotational_acceleration(false);  // in global frame

    // pass info to arrays for AeroDyn
    for (int i = 0; i < 3; i++) {
        pImpl->HubPos[i] = hubPos[i];
        pImpl->HubVel[i] = hubTranVel[i];
        pImpl->HubVel[i + 3] = hubRotVel[i];
        pImpl->HubAcc[i] = hubTranAcc[i];
        pImpl->HubAcc[i + 3] = hubRotAcc[i];
        pImpl->HubOri[i * 3 + 0] = hubOri(i, 0);
        pImpl->HubOri[i * 3 + 1] = hubOri(i, 1);
        pImpl->HubOri[i * 3 + 2] = hubOri(i, 2);
    }
}

void seahowl::aero::AeroDynAdapter::update_nacelle_motion(seahowl::aero::TurbineAero& turbine) {
    // Get the information about nacelle
    auto& nac = turbine.rna.body_nacelle;
    auto nacPos = nac.get_position();
    auto nacOri = nac.get_rotation().toRotationMatrix();  // get a rotation matrix 3x3
    auto nacTranVel = nac.get_velocity();
    auto nacRotVel = nac.get_rotational_velocity(false);  // in global frame
    auto nacTranAcc = nac.get_acceleration();
    auto nacRotAcc = nac.get_rotational_acceleration(false);  // in global frame

    // pass info to arrays for AeroDyn
    for (int i = 0; i < 3; i++) {
        pImpl->NacPos[i] = nacPos[i];
        pImpl->NacVel[i] = nacTranVel[i];
        pImpl->NacVel[i + 3] = nacRotVel[i];
        pImpl->NacAcc[i] = nacTranAcc[i];
        pImpl->NacAcc[i + 3] = nacRotAcc[i];
        pImpl->NacOri[i * 3 + 0] = nacOri(i, 0);
        pImpl->NacOri[i * 3 + 1] = nacOri(i, 1);
        pImpl->NacOri[i * 3 + 2] = nacOri(i, 2);
    }
}

void seahowl::aero::AeroDynAdapter::update_roots_motion(seahowl::aero::TurbineAero& turbine) {
    auto nblades = turbine.rna.rotor->blades.size();
    pImpl->NumBlades = nblades;

    for (int i = 0; i < nblades; i++) {
        auto& blade = turbine.rna.rotor->blades[i];
        auto& bldRoot = blade->body_root;
        auto bldRootPos = bldRoot->get_position();
        auto bldRootOri = bldRoot->get_rotation().toRotationMatrix();
        auto bldRootTranVel = bldRoot->get_velocity();
        auto bldRootRotVel = bldRoot->get_rotational_velocity(false);  // in global frame
        auto bldRootTranAcc = bldRoot->get_acceleration();
        auto bldRootRotAcc = bldRoot->get_rotational_acceleration(false);  // in global frame

        // pass info to arrays for AeroDyn
        for (int j = 0; j < 3; j++) {
            int p = i * 3 + j;
            int q = i * 6 + j;
            pImpl->BldRootPos[p] = bldRootPos[j];
            pImpl->BldRootVel[q] = bldRootTranVel[j];
            pImpl->BldRootVel[q + 3] = bldRootRotVel[j];
            pImpl->BldRootAcc[q] = bldRootTranAcc[j];
            pImpl->BldRootAcc[q + 3] = bldRootRotAcc[j];
            pImpl->BldRootOri[i * 9 + j * 3 + 0] = bldRootOri(j, 0);
            pImpl->BldRootOri[i * 9 + j * 3 + 1] = bldRootOri(j, 1);
            pImpl->BldRootOri[i * 9 + j * 3 + 2] = bldRootOri(j, 2);
        }
    }
}

void seahowl::aero::AeroDynAdapter::update_mesh_motion(seahowl::aero::TurbineAero& turbine) {
    auto nblades = turbine.rna.rotor->blades.size();
    auto nMeshPerBlade = turbine.rna.rotor->blades[0]->nodes.size();
    auto nMesh = nMeshPerBlade * nblades;
    pImpl->NumMeshPts = nMesh;

    for (int i = 0; i < nblades; i++) {
        for (int j = 0; j < nMeshPerBlade; j++) {
            auto& bldMesh = turbine.rna.rotor->blades[i]->nodes[j];
            auto meshPos = bldMesh.get_position();
            auto meshOri = bldMesh.get_rotation().toRotationMatrix();
            auto meshTranVel = bldMesh.get_velocity();
            auto meshRotVel = bldMesh.get_rotational_velocity(false);  // in global frame
            auto meshTranAcc = bldMesh.get_acceleration();
            auto meshRotAcc = bldMesh.get_rotational_acceleration(false);  // in global frame

            auto ii = i * nMeshPerBlade + j;
            for (int k = 0; k < 3; k++) {
                int p = ii * 3 + k;
                int q = ii * 6 + k;
                pImpl->MeshPos[p] = meshPos[k];
                pImpl->MeshVel[q] = meshTranVel[k];
                pImpl->MeshVel[q + 3] = meshRotVel[k];
                pImpl->MeshAcc[q] = meshTranAcc[k];
                pImpl->MeshAcc[q + 3] = meshRotAcc[k];
                pImpl->MeshOri[ii * 9 + k * 3 + 0] = meshOri(k, 0);
                pImpl->MeshOri[ii * 9 + k * 3 + 1] = meshOri(k, 1);
                pImpl->MeshOri[ii * 9 + k * 3 + 2] = meshOri(k, 2);
            }
        }
    }
}

void seahowl::aero::AeroDynInflowLib::CheckError() {
    if (ErrStat == 0) {
        return;
    } else if (ErrStat == 1) {
        spdlog::info("AeroDyn/InflowWind INFO: {}.", ErrMsg);
    } else if (ErrStat == 2) {
        spdlog::warn("AeroDyn/InflowWind WARNING: {}", ErrMsg);
    } else {
        spdlog::error("AeroDyn/InflowWind ERROR: {}.", ErrMsg);
    }
}

void seahowl::aero::AeroDynInflowLib::set_aerodyn_infile(const std::string& name) {
    spdlog::debug("Set AeroDyn INFILE: {}.", name);
    ADinputFileString = name;
    ADinputFileStringLength = ADinputFileString.length();
}

void seahowl::aero::AeroDynInflowLib::set_inflowwind_infile(const std::string& name) {
    spdlog::debug("Set InflowWind INFILE: {}.", name);
    IfWinputFileString = name;
    IfWinputFileStringLength = IfWinputFileString.length();
}

void seahowl::aero::AeroDynInflowLib::set_outfile_name(const std::string& name) {
    spdlog::debug("Set AeroDyn/InflowWind output file: {}.", name);
    strcpy(OutRootName, name.c_str());
}

void seahowl::aero::AeroDynInflowLib::set_time(double time) {
    Time = time;
    TimeLast = Time - DT;
}

void seahowl::aero::AeroDynInflowLib::initialize_arrays(int NumBlades, int NumMeshPts) {
    //
    this->NumBlades = NumBlades;
    this->NumMeshPts = NumMeshPts;

    // hub
    HubPos = new float[3];
    HubOri = new double[9];
    HubVel = new float[6];
    HubAcc = new float[6];

    // nacelle
    NacPos = new float[3];
    NacOri = new double[9];
    NacVel = new float[6];
    NacAcc = new float[6];

    // blade roots
    BldRootPos = new float[3 * NumBlades];
    BldRootOri = new double[9 * NumBlades];
    BldRootVel = new float[6 * NumBlades];
    BldRootAcc = new float[6 * NumBlades];

    // blades
    MeshPos = new float[3 * NumMeshPts];
    MeshOri = new double[9 * NumMeshPts];
    MeshVel = new float[6 * NumMeshPts];
    MeshAcc = new float[6 * NumMeshPts];
    MeshFrc = new float[6 * NumMeshPts];
    MeshPtToBladeNum = new int[NumMeshPts];
}

void seahowl::aero::AeroDynInflowLib::Init() {
    // input files
    const char* ADinputFile = ADinputFileString.c_str();
    const char* IfWinputFile = IfWinputFileString.c_str();

    gravity = 9.80665;       // Gravitational acceleration (m/s^2)
    defFldDens = 1.225;      // Air density (kg/m^3)
    defKinVisc = 1.464E-05;  // Kinematic viscosity of working fluid (m^2/s)
    defSpdSound = 335.0;     // Speed of sound in working fluid (m/s)
    defPatm = 103500.0;      // Atmospheric pressure (Pa) [used only for an MHK turbine cavitation check]
    defPvap = 1700.0;        // Vapour pressure of working fluid (Pa) [used only for an MHK turbine cavitation check]
    WtrDpth = 0.0;           // Water depth (m)
    MSL2SWL = 0.0;           // Offset between still-water level and mean sea level (m) [positive upward]

    // Type of aerodynamic projection
    AeroProjMod = 1;  // Original AeroDyn model where momentum balance is done in the WithoutSweepPitchTwist system

    // Interpolation order (must be 1: linear, or 2: quadratic)
    InterpOrder = 1;  // default of linear interpolation

    // Initial time related variables
    // Time         = 0.0; // initial time
    // DT           = 0.1; // typical default for AD
    TMax = 180;  // typical default for AD

    VTKNacDim =
        new float[6]{0,     -4.2751, -4.2751, 12,
                     8.552, 8.552};  // default nacelle dimension for VTK surface rendering [x0,y0,z0,Lx,Ly,Lz] (m)
    VTKHubRad = 3.97;                // default hub radius for VTK surface rendering

    // NumBlades    = 3;
    // NumMeshPts   = 1;

    // Output file
    wrOuts = 0;     // wrOuts -- file format for writing outputs
    DT_Outs = 0.0;  // DT_Outs -- timestep for outputs to file

    ADI_C_PreInit(NumTurbines, TransposeDCM, PointLoadOutput_in, DebugLevel_in, ErrStat, ErrMsg);
    CheckError();

    TurbOrigin = new float[3]{0.0, 0.0, 0.0};

    ADI_C_SetupRotor(iWT, TurbineIsHAWT, TurbOrigin, HubPos, HubOri, NacPos, NacOri, NumBlades, BldRootPos, BldRootOri,
                     NumMeshPts, MeshPos, MeshOri, MeshPtToBladeNum, ErrStat, ErrMsg);
    CheckError();

    char OutVTKDir[1024] = "./output";  // to change to actual output folder from SEAHOWL OutputManager
    ADI_C_Init(ADinputFilePassed, &ADinputFile, ADinputFileStringLength, IfWinputFilePassed, &IfWinputFile,
               IfWinputFileStringLength, OutRootName, OutVTKDir, gravity, defFldDens, defKinVisc, defSpdSound, defPatm,
               defPvap, WtrDpth, MSL2SWL, InterpOrder, DT, TMax, storeHHVel, WrVTK, WrVTK_Type, WrVTK_dt, VTKNacDim,
               VTKHubRad, wrOuts, DT_Outs, NumChannels, OutputChannelNames, OutputChannelUnits, ErrStat, ErrMsg);
    CheckError();
}

void seahowl::aero::AeroDynInflowLib::Calcul() {
    ADI_C_CalcOutput(Time, OutputChannelValues, ErrStat, ErrMsg);
    CheckError();

    // ADI_C_GetRotorLoads(...);
}

void seahowl::aero::AeroDynInflowLib::Update() {
    ADI_C_SetRotorMotion(iWT, HubPos, HubOri, HubVel, HubAcc, NacPos, NacOri, NacVel, NacAcc, BldRootPos, BldRootOri,
                         BldRootVel, BldRootAcc, NumMeshPts, MeshPos, MeshOri, MeshVel, MeshAcc, ErrStat, ErrMsg);
    CheckError();

    ADI_C_UpdateStates(TimeLast, Time, ErrStat, ErrMsg);
    CheckError();
}

void seahowl::aero::AeroDynInflowLib::End() {
    ADI_C_End(ErrStat, ErrMsg);
    CheckError();

    delete[] HubPos, HubOri, HubVel, HubAcc;
    delete[] NacPos, NacOri, NacVel, NacAcc;
    delete[] BldRootPos, BldRootOri, BldRootVel, BldRootAcc;
    delete[] MeshPos, MeshOri, MeshVel, MeshAcc;
}

seahowl::aero::TurbineAeroDyn::TurbineAeroDyn() : TurbineAero() {
    rna.rotor = std::make_shared<seahowl::aero::RotorAeroDyn>(tower);
}

void seahowl::aero::TurbineAeroDyn::initialize(double time, double dt) {
    TurbineAero::initialize(time, dt);

    // VTK options for AeroDyn
    aerodyn.pImpl->WrVTK = WrVTK;
    aerodyn.pImpl->WrVTK_Type = WrVTK_Type;
    aerodyn.pImpl->WrVTK_dt;

    // initialize AeroDyn adapter
    aerodyn.initialize(time, dt, *this);
}

void seahowl::aero::TurbineAeroDyn::compute_fluid_loads(const seahowl::env::FluidModel& wind_model, double time) {
    aerodyn.compute_loads(time, *this);
    dynamic_cast<seahowl::aero::RotorAeroDyn&>(*rna.rotor).loads_aerodyn = aerodyn.pImpl->MeshFrc;
    rna.compute_fluid_loads(wind_model, time);
    if (foundation) {
        foundation->compute_fluid_loads(wind_model, time);
    }
}

seahowl::aero::RotorAeroDyn::RotorAeroDyn(TowerAero& tower_ref) : RotorAeroBEMT(tower_ref) {}

void seahowl::aero::RotorAeroDyn::compute_fluid_loads(const seahowl::env::FluidModel& wind_model, double time) {
    // get loads from AeroDyn
    int count_blade = -1;
    for (auto& blade : blades) {
        count_blade += 1;
        int count_node = -1;
        for (auto& node : blade->nodes) {
            count_node += 1;
            // store load in global frame
            int pp = (count_blade * (blade->elements.size() + 1) + count_node) * 6;
            node.load = Vector3d(loads_aerodyn[pp], loads_aerodyn[pp + 1], loads_aerodyn[pp + 2]);
            node.moment = Vector3d(loads_aerodyn[pp + 3], loads_aerodyn[pp + 4], loads_aerodyn[pp + 5]);
        }
        // update loads of blade
        for (int ii = 0; ii < blade->elements.size(); ii++) {
            blade->loads[ii] = blade->elements[ii].get_load();
            blade->moments[ii] = blade->elements[ii].get_moment();
        }
    }
}
