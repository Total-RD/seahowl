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

seahowl::aero::AeroDynAdapter::AeroDynAdapter() {
    spdlog::debug("Initialising Aerodyn15 adapter");
    pImpl = AeroDynInflowLib();
    pImpl.ADinputFilePassed = false;
    pImpl.IfWinputFilePassed = false;
    pImpl.SetOUTNAME("Turbine");
}

seahowl::aero::AeroDynAdapter::~AeroDynAdapter() {}

void seahowl::aero::AeroDynAdapter::set_infiles(const std::string& AerodynInfile, const std::string& InflowInfile) {
    pImpl.SetADINFILE(AerodynInfile);
    pImpl.SetIFWINFILE(InflowInfile);
}

void seahowl::aero::AeroDynAdapter::initialize(double time, double dt, seahowl::aero::TurbineAero& turbine) {
    pImpl.SetTimeStep(dt);
    pImpl.SetTime(time);
    update_turbine_variables(turbine);

    // init blade indexing on AeroDyn mesh
    // this should be moved in a separate function
    int npoints = 0;
    for (auto& blade : turbine.rna.rotor->blades) {
        npoints += blade->nodes.size();
    }
    pImpl.MeshPtToBladeNum = new int[npoints];
    int idx_blade = 0;
    for (auto& blade : turbine.rna.rotor->blades) {
        int idx_node = 0;
        for (auto& node : blade->nodes) {
            pImpl.MeshPtToBladeNum[idx_node] = idx_blade + 1;
            idx_node += 1;
        }
        idx_blade += 1;
    }

    pImpl.Init();
}

void seahowl::aero::AeroDynAdapter::calcul(double time, seahowl::aero::TurbineAero& turbine) {
    pImpl.SetTime(time);
    update_turbine_variables(turbine);
    if (time > 0.) {
        pImpl.Update();
        pImpl.Calcul();
    } else {
        pImpl.Calcul();
    }
}

void seahowl::aero::AeroDynAdapter::update(double time, double dt, seahowl::aero::TurbineAero& turbine) {
    pImpl.SetTime(time);
    pImpl.SetTimeNext(time + dt);
    update_turbine_variables(turbine);
    pImpl.Update();
}

void seahowl::aero::AeroDynAdapter::end(double time, double dt, seahowl::aero::TurbineAero& turbine) {
    pImpl.End();
}

void seahowl::aero::AeroDynAdapter::update_turbine_variables(seahowl::aero::TurbineAero& turbine) {
    setMotionHub(turbine);
    setMotionNac(turbine);
    setMotionRoot(turbine);
    setMotionMesh(turbine);
}

void seahowl::aero::AeroDynAdapter::setMotionHub(seahowl::aero::TurbineAero& turbine) {
    float* hubPos_C = new float[3];
    double* hubOri_C = new double[9];
    float* hubVel_C = new float[6];
    float* hubAcc_C = new float[6];

    // Get the information about hub
    auto& hub = turbine.rna.rotor->body_hub;
    auto hubPos = hub.get_position();
    auto hubOri = hub.get_rotation().toRotationMatrix();  // get a rotation matrix 3x3
    auto hubTranVel = hub.get_velocity();
    auto hubRotVel = hub.get_rotational_velocity(false);  // in global frame
    auto hubTranAcc = hub.get_acceleration();
    auto hubRotAcc = hub.get_rotational_acceleration(false);  // in global frame

    for (int i = 0; i < 3; i++) {
        hubPos_C[i] = hubPos[i];
        hubVel_C[i] = hubTranVel[i];
        hubVel_C[i + 3] = hubRotVel[i];
        hubAcc_C[i] = hubTranAcc[i];
        hubAcc_C[i + 3] = hubRotAcc[i];
    }

    hubOri_C[0] = hubOri(0, 0);
    hubOri_C[1] = hubOri(0, 1);
    hubOri_C[2] = hubOri(0, 2);
    hubOri_C[3] = hubOri(1, 0);
    hubOri_C[4] = hubOri(1, 1);
    hubOri_C[5] = hubOri(1, 2);
    hubOri_C[6] = hubOri(2, 0);
    hubOri_C[7] = hubOri(2, 1);
    hubOri_C[8] = hubOri(2, 2);

    pImpl.SetHubPos(hubPos_C);
    pImpl.SetHubOri(hubOri_C);
    pImpl.SetHubVel(hubVel_C);
    pImpl.SetHubAcc(hubAcc_C);
}

void seahowl::aero::AeroDynAdapter::setMotionNac(seahowl::aero::TurbineAero& turbine) {
    float* nacPos_C = new float[3];
    double* nacOri_C = new double[9];
    float* nacVel_C = new float[6];
    float* nacAcc_C = new float[6];

    // Get the information about nacelle
    auto& nac = turbine.rna.body_nacelle;
    auto nacPos = nac.get_position();
    auto nacOri = nac.get_rotation().toRotationMatrix();  // get a rotation matrix 3x3
    auto nacTranVel = nac.get_velocity();
    auto nacRotVel = nac.get_rotational_velocity(false);  // in global frame
    auto nacTranAcc = nac.get_acceleration();
    auto nacRotAcc = nac.get_rotational_acceleration(false);  // in global frame

    for (int i = 0; i < 3; i++) {
        nacPos_C[i] = nacPos[i];
        nacVel_C[i] = nacTranVel[i];
        nacVel_C[i + 3] = nacRotVel[i];
        nacAcc_C[i] = nacTranAcc[i];
        nacAcc_C[i + 3] = nacRotAcc[i];
    }

    nacOri_C[0] = nacOri(0, 0);
    nacOri_C[1] = nacOri(0, 1);
    nacOri_C[2] = nacOri(0, 2);
    nacOri_C[3] = nacOri(1, 0);
    nacOri_C[4] = nacOri(1, 1);
    nacOri_C[5] = nacOri(1, 2);
    nacOri_C[6] = nacOri(2, 0);
    nacOri_C[7] = nacOri(2, 1);
    nacOri_C[8] = nacOri(2, 2);

    pImpl.SetNacPos(nacPos_C);
    pImpl.SetNacOri(nacOri_C);
    pImpl.SetNacVel(nacVel_C);
    pImpl.SetNacAcc(nacAcc_C);
}

void seahowl::aero::AeroDynAdapter::setMotionRoot(seahowl::aero::TurbineAero& turbine) {
    auto nblades = turbine.rna.rotor->blades.size();
    float* bldRootPos_C = new float[3 * nblades];
    double* bldRootOri_C = new double[9 * nblades];
    float* bldRootVel_C = new float[6 * nblades];
    float* bldRootAcc_C = new float[6 * nblades];

    for (int i = 0; i < nblades; i++) {
        auto& blade = turbine.rna.rotor->blades[i];
        auto& bldRoot = blade->body_root;
        auto bldRootPos = bldRoot->get_position();
        auto bldRootOri = bldRoot->get_rotation().toRotationMatrix();
        auto bldRootTranVel = bldRoot->get_velocity();
        auto bldRootRotVel = bldRoot->get_rotational_velocity(false);  // in global frame
        auto bldRootTranAcc = bldRoot->get_acceleration();
        auto bldRootRotAcc = bldRoot->get_rotational_acceleration(false);  // in global frame

        for (int j = 0; j < 3; j++) {
            int p = i * 3 + j;
            int q = i * 6 + j;
            bldRootPos_C[p] = bldRootPos[j];
            bldRootVel_C[q] = bldRootTranVel[j];
            bldRootVel_C[q + 3] = bldRootRotVel[j];
            bldRootAcc_C[q] = bldRootTranAcc[j];
            bldRootAcc_C[q + 3] = bldRootRotAcc[j];
        }

        bldRootOri_C[i * 9] = bldRootOri(0, 0);
        bldRootOri_C[i * 9 + 1] = bldRootOri(0, 1);
        bldRootOri_C[i * 9 + 2] = bldRootOri(0, 2);
        bldRootOri_C[i * 9 + 3] = bldRootOri(1, 0);
        bldRootOri_C[i * 9 + 4] = bldRootOri(1, 1);
        bldRootOri_C[i * 9 + 5] = bldRootOri(1, 2);
        bldRootOri_C[i * 9 + 6] = bldRootOri(2, 0);
        bldRootOri_C[i * 9 + 7] = bldRootOri(2, 1);
        bldRootOri_C[i * 9 + 8] = bldRootOri(2, 2);
    }

    pImpl.SetNumBlades(nblades);
    pImpl.SetBldRootPos(bldRootPos_C);
    pImpl.SetBldRootOri(bldRootOri_C);
    pImpl.SetBldRootVel(bldRootVel_C);
    pImpl.SetBldRootAcc(bldRootAcc_C);
}

void seahowl::aero::AeroDynAdapter::setMotionMesh(seahowl::aero::TurbineAero& turbine) {
    auto nblades = turbine.rna.rotor->blades.size();
    auto nMeshPerBlade = turbine.rna.rotor->blades[0]->nodes.size();
    auto nMesh = nMeshPerBlade * nblades;
    float* meshPos_C = new float[3 * nMesh];
    double* meshOri_C = new double[9 * nMesh];
    float* meshVel_C = new float[6 * nMesh];
    float* meshAcc_C = new float[6 * nMesh];

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
                meshPos_C[p] = meshPos[k];
                meshVel_C[q] = meshTranVel[k];
                meshVel_C[q + 3] = meshRotVel[k];
                meshAcc_C[q] = meshTranAcc[k];
                meshAcc_C[q + 3] = meshRotAcc[k];
            }

            meshOri_C[ii * 9] = meshOri(0, 0);
            meshOri_C[ii * 9 + 1] = meshOri(0, 1);
            meshOri_C[ii * 9 + 2] = meshOri(0, 2);
            meshOri_C[ii * 9 + 3] = meshOri(1, 0);
            meshOri_C[ii * 9 + 4] = meshOri(1, 1);
            meshOri_C[ii * 9 + 5] = meshOri(1, 2);
            meshOri_C[ii * 9 + 6] = meshOri(2, 0);
            meshOri_C[ii * 9 + 7] = meshOri(2, 1);
            meshOri_C[ii * 9 + 8] = meshOri(2, 2);
        }
    }

    pImpl.SetNumMeshPts(nMesh);
    pImpl.SetMeshPos(meshPos_C);
    pImpl.SetMeshOri(meshOri_C);
    pImpl.SetMeshVel(meshVel_C);
    pImpl.SetMeshAcc(meshAcc_C);
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

void seahowl::aero::AeroDynInflowLib::SetADINFILE(const std::string& name) {
    spdlog::debug("Set AeroDyn INFILE: {}.", name);
    ADinputFileString = name;
    ADinputFileStringLength = ADinputFileString.length();
}

void seahowl::aero::AeroDynInflowLib::SetIFWINFILE(const std::string& name) {
    spdlog::debug("Set InflowWind INFILE: {}.", name);
    IfWinputFileString = name;
    IfWinputFileStringLength = IfWinputFileString.length();
}

void seahowl::aero::AeroDynInflowLib::SetOUTNAME(const std::string& name) {
    spdlog::debug("Set AeroDyn/InflowWind output file: {}.", name);
    strcpy(OutRootName, name.c_str());
}

void seahowl::aero::AeroDynInflowLib::SetTime(double time) {
    Time = time;
    TimeLast = Time - DT;
}

void seahowl::aero::AeroDynInflowLib::SetTimeStep(double dt) {
    DT = dt;
}

void seahowl::aero::AeroDynInflowLib::SetTimeNext(double timenext) {
    TimeNext = timenext;
}

void seahowl::aero::AeroDynInflowLib::SetVTK(int SaveVTK, int VTK_type, double VTK_dt) {
    WrVTK = SaveVTK;
    WrVTK_Type = VTK_type;
    WrVTK_dt = VTK_dt;
}

void seahowl::aero::AeroDynInflowLib::SetHubPos(float* hubPos) {
    HubPos = hubPos;
}

void seahowl::aero::AeroDynInflowLib::SetHubOri(double* hubOri) {
    HubOri = hubOri;
}

void seahowl::aero::AeroDynInflowLib::SetHubVel(float* hubVel) {
    HubVel = hubVel;
}

void seahowl::aero::AeroDynInflowLib::SetHubAcc(float* hubAcc) {
    HubAcc = hubAcc;
}

void seahowl::aero::AeroDynInflowLib::SetNacPos(float* nacPos) {
    NacPos = nacPos;
}

void seahowl::aero::AeroDynInflowLib::SetNacOri(double* nacOri) {
    NacOri = nacOri;
}

void seahowl::aero::AeroDynInflowLib::SetNacVel(float* nacVel) {
    NacVel = nacVel;
}

void seahowl::aero::AeroDynInflowLib::SetNacAcc(float* nacAcc) {
    NacAcc = nacAcc;
}

void seahowl::aero::AeroDynInflowLib::SetNumBlades(int nBlades) {
    NumBlades = nBlades;
}

void seahowl::aero::AeroDynInflowLib::SetBldRootPos(float* bldRootPos) {
    BldRootPos = bldRootPos;
}

void seahowl::aero::AeroDynInflowLib::SetBldRootOri(double* bldRootOri) {
    BldRootOri = bldRootOri;
}

void seahowl::aero::AeroDynInflowLib::SetBldRootVel(float* bldRootVel) {
    BldRootVel = bldRootVel;
}

void seahowl::aero::AeroDynInflowLib::SetBldRootAcc(float* bldRootAcc) {
    BldRootAcc = bldRootAcc;
}

void seahowl::aero::AeroDynInflowLib::SetNumMeshPts(int nMeshPtsAllBlades) {
    NumMeshPts = nMeshPtsAllBlades;
}

void seahowl::aero::AeroDynInflowLib::SetMeshPos(float* meshPosAllBlades) {
    MeshPos = meshPosAllBlades;
}

void seahowl::aero::AeroDynInflowLib::SetMeshOri(double* meshOriAllBlades) {
    MeshOri = meshOriAllBlades;
}

void seahowl::aero::AeroDynInflowLib::SetMeshVel(float* meshVelAllBlades) {
    MeshVel = meshVelAllBlades;
}

void seahowl::aero::AeroDynInflowLib::SetMeshAcc(float* meshAccAllBlades) {
    MeshAcc = meshAccAllBlades;
}

void seahowl::aero::AeroDynInflowLib::SetAeroLoads(float* meshFrcAllBlades) {
    MeshFrc = meshFrcAllBlades;
}

/* FIXME: add routines
void seahowl::aero::AeroDynInflowLib::PreInit() {
    ADI_C_PreInit()
}

void seahowl::aero::AeroDynInflowLib::SetupRotor() {
    ADI_C_SetupRotor()
}

void seahowl::aero::AeroDynInflowLib::SetRotorMotion() {
    ADI_C_SetRotorMotion()
}

void seahowl::aero::AeroDynInflowLib::GetRotorLoads() {
    ADI_C_GetRotorLoads()
}

void seahowl::aero::AeroDynInflowLib::GetDiskAvgVel() {
    ADI_C_GetDiskAvgVel()
}
*/

void seahowl::aero::AeroDynInflowLib::Init() {
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

    TurbOrigin = new float[3]{0.0, 0.0, 0.0};

    ADI_C_SetupRotor(iWT, TurbineIsHAWT, TurbOrigin, HubPos, HubOri, NacPos, NacOri, NumBlades, BldRootPos, BldRootOri,
                     NumMeshPts, MeshPos, MeshOri, MeshPtToBladeNum, ErrStat, ErrMsg);

    char OutVTKDir[1024] = "./output";  // to change to actual output folder from SEAHOWL OutputManager
    ADI_C_Init(ADinputFilePassed, &ADinputFile, ADinputFileStringLength, IfWinputFilePassed, &IfWinputFile,
               IfWinputFileStringLength, OutRootName, OutVTKDir, gravity, defFldDens, defKinVisc, defSpdSound, defPatm,
               defPvap, WtrDpth, MSL2SWL, InterpOrder, DT, TMax, storeHHVel, WrVTK, WrVTK_Type, WrVTK_dt, VTKNacDim,
               VTKHubRad, wrOuts, DT_Outs, NumChannels, OutputChannelNames, OutputChannelUnits, ErrStat, ErrMsg);

    CheckError();
}

void seahowl::aero::AeroDynInflowLib::Calcul() {
    float* OutputChannelValues = new float[6 * NumMeshPts];
    ADI_C_CalcOutput(Time, OutputChannelValues, ErrStat, ErrMsg);

    SetAeroLoads(OutputChannelValues);

    CheckError();
}

void seahowl::aero::AeroDynInflowLib::Update() {
    ADI_C_SetRotorMotion(iWT, HubPos, HubOri, HubVel, HubAcc, NacPos, NacOri, NacVel, NacAcc, BldRootPos, BldRootOri,
                         BldRootVel, BldRootAcc, NumMeshPts, MeshPos, MeshOri, MeshVel, MeshAcc, ErrStat, ErrMsg);

    ADI_C_UpdateStates(TimeLast, Time, ErrStat, ErrMsg);

    CheckError();
}

void seahowl::aero::AeroDynInflowLib::End() {
    ADI_C_End(ErrStat, ErrMsg);
    CheckError();

    // delete [] hubPos_C, hubOri_C, hubVel_C, hubAcc_C;
    // delete [] nacPos_C, nacOri_C, nacVel_C, nacAcc_C;
    // delete [] bldRootPos_C, bldRootOri_C, bldRootVel_C, bldRootAcc_C;
    // delete [] meshPos_C, meshOri_C, meshVel_C, meshAcc_C;
}

seahowl::aero::TurbineAeroDyn::TurbineAeroDyn() : TurbineAero() {
    rna.rotor = std::make_shared<seahowl::aero::RotorAeroDyn>(tower);
}

void seahowl::aero::TurbineAeroDyn::initialize(double time, double dt) {
    TurbineAero::initialize(time, dt);

    aerodyn.pImpl.SetVTK(WrVTK, WrVTK_Type, WrVTK_dt);
    aerodyn.initialize(time, dt, *this);
}

void seahowl::aero::TurbineAeroDyn::compute_fluid_loads(const seahowl::env::FluidModel& wind_model, double time) {
    aerodyn.calcul(time, *this);
    dynamic_cast<seahowl::aero::RotorAeroDyn&>(*rna.rotor).loads_aerodyn = aerodyn.pImpl.MeshFrc;
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
