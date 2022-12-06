#include "seahowl/aero/aerodyn.h"

#include <stdexcept>
#include <vector>
#include <string>
#include <iostream>
#include <fstream>
// #include <numeric>
// #include <sstream>

seahowl::aero::AeroDyn::AeroDyn(std::string AerodynInfile, std::string InflowInfile) {
    std::cout << "Initialising Aerodyn15" << std::endl;

    pImpl.ADinputFilePassed  = false;
    pImpl.IfWinputFilePassed = false;

    pImpl.SetADINFILE(AerodynInfile);
    pImpl.SetIFWINFILE(InflowInfile);

    pImpl.SetOUTNAME("Output_ADIlib_default");
}

seahowl::aero::AeroDyn::~AeroDyn() {}

void seahowl::aero::AeroDyn::init(double time, double dt, seahowl::core::Turbine& turbine) {
    pImpl.SetTimeStep(dt);
    pImpl.SetTime(time);
    update_turbine_variables(turbine);
    pImpl.Init();
}

void seahowl::aero::AeroDyn::calcul(double time, seahowl::core::Turbine& turbine) {
    pImpl.SetTime(time);
    update_turbine_variables(turbine);
    if (time == 0.) {
        pImpl.Calcul();
    } else {
        pImpl.Update();
        pImpl.Calcul();
    }
}

void seahowl::aero::AeroDyn::update(double time, double dt, seahowl::core::Turbine& turbine) {
    pImpl.SetTime(time);
    pImpl.SetTimeNext(time+dt);
    update_turbine_variables(turbine);
    pImpl.Update();
}

void seahowl::aero::AeroDyn::end(double time, double dt, seahowl::core::Turbine& turbine) {
    pImpl.End();
}

void seahowl::aero::AeroDyn::update_turbine_variables(seahowl::core::Turbine& turbine) {
    setMotionHub(turbine);
    setMotionNac(turbine);
    setMotionRoot(turbine);
    setMotionMesh(turbine);
}

void seahowl::aero::AeroDyn::setMotionHub(seahowl::core::Turbine& turbine) {
    float  *hubPos_C = new float  [3];
    double *hubOri_C = new double [9];
    float  *hubVel_C = new float  [6];
    float  *hubAcc_C = new float  [6];

    // Get the information about hub
    auto hubPos     = turbine.rotor.elasto.body_hub->GetPos();
    auto hubOri     = turbine.rotor.elasto.body_hub->GetA(); // get a rotation matrix 3x3 
    auto hubTranVel = turbine.rotor.elasto.body_hub->GetPos_dt();
    auto hubRotVel  = turbine.rotor.elasto.body_hub->GetWvel_par();
    auto hubTranAcc = turbine.rotor.elasto.body_hub->GetPos_dtdt();
    auto hubRotAcc  = turbine.rotor.elasto.body_hub->GetWacc_par();

    for (int i = 0; i < 3; i++) {
        hubPos_C[i]   = hubPos[i]; 
        hubVel_C[i]   = hubTranVel[i];
        hubVel_C[i+3] = hubRotVel[i];
        hubAcc_C[i]   = hubTranAcc[i];
        hubAcc_C[i+3] = hubRotAcc[i];
    }

    // rotate the local coordinate system from seahowl to aerodyn
    hubOri  =  hubOri * chrono::ChMatrix33(Q_from_AngAxis(-chrono::CH_C_PI/2,chrono::VECT_Y));

    hubOri_C[0] = hubOri(0,0);
    hubOri_C[1] = hubOri(0,1);
    hubOri_C[2] = hubOri(0,2);
    hubOri_C[3] = hubOri(1,0);
    hubOri_C[4] = hubOri(1,1);
    hubOri_C[5] = hubOri(1,2);
    hubOri_C[6] = hubOri(2,0);
    hubOri_C[7] = hubOri(2,1);
    hubOri_C[8] = hubOri(2,2);

    pImpl.SetHubPos(hubPos_C);
    pImpl.SetHubOri(hubOri_C);
    pImpl.SetHubVel(hubVel_C);
    pImpl.SetHubAcc(hubAcc_C);
}

void seahowl::aero::AeroDyn::setMotionNac(seahowl::core::Turbine& turbine) {
    float  *nacPos_C = new float  [3];
    double *nacOri_C = new double [9];
    float  *nacVel_C = new float  [6];
    float  *nacAcc_C = new float  [6];

    // Get the information about nacelle
    auto nacPos     = turbine.rotor.elasto.body_nacelle->GetPos();
    auto nacOri     = turbine.rotor.elasto.body_nacelle->GetA(); // get a rotation matrix 3x3 
    auto nacTranVel = turbine.rotor.elasto.body_nacelle->GetPos_dt();
    auto nacRotVel  = turbine.rotor.elasto.body_nacelle->GetWvel_par();
    auto nacTranAcc = turbine.rotor.elasto.body_nacelle->GetPos_dtdt();
    auto nacRotAcc  = turbine.rotor.elasto.body_nacelle->GetWacc_par();
 
    for (int i = 0; i < 3; i++) {
        nacPos_C[i]   = nacPos[i];
        nacVel_C[i]   = nacTranVel[i];
        nacVel_C[i+3] = nacRotVel[i];
        nacAcc_C[i]   = nacTranAcc[i];
        nacAcc_C[i+3] = nacRotAcc[i];
    }    

    nacOri_C[0] = nacOri(0,0);
    nacOri_C[1] = nacOri(0,1);
    nacOri_C[2] = nacOri(0,2);
    nacOri_C[3] = nacOri(1,0);
    nacOri_C[4] = nacOri(1,1);
    nacOri_C[5] = nacOri(1,2);
    nacOri_C[6] = nacOri(2,0);
    nacOri_C[7] = nacOri(2,1);
    nacOri_C[8] = nacOri(2,2);

    pImpl.SetNacPos(nacPos_C);
    pImpl.SetNacOri(nacOri_C);
    pImpl.SetNacVel(nacVel_C);
    pImpl.SetNacAcc(nacAcc_C);
}

void seahowl::aero::AeroDyn::setMotionRoot(seahowl::core::Turbine& turbine) {
    auto nblades = turbine.blades.size();
    float  *bldRootPos_C = new float  [3 * nblades];
    double *bldRootOri_C = new double [9 * nblades];
    float  *bldRootVel_C = new float  [6 * nblades];
    float  *bldRootAcc_C = new float  [6 * nblades];

    for (int i = 0; i < nblades; i++) {
        auto bldRootPos     = turbine.rotor.blades[i]->elasto->nodes[0]->GetPos();
        auto bldRootOri     = turbine.rotor.blades[i]->elasto->nodes[0]->GetA();
        auto bldRootTranVel = turbine.rotor.blades[i]->elasto->nodes[0]->GetPos_dt();
        auto bldRootRotVel  = turbine.rotor.blades[i]->elasto->nodes[0]->GetWvel_par();
        auto bldRootTranAcc = turbine.rotor.blades[i]->elasto->nodes[0]->GetPos_dtdt();
        auto bldRootRotAcc  = turbine.rotor.blades[i]->elasto->nodes[0]->GetWacc_par();
        for (int j = 0; j < 3; j++) {
            int p = i*3 + j;
            int q = i*6 + j;
            bldRootPos_C[p]   = bldRootPos[j];
            bldRootVel_C[q]   = bldRootTranVel[j];
            bldRootVel_C[q+3] = bldRootRotVel[j];
            bldRootAcc_C[q]   = bldRootTranAcc[j];
            bldRootAcc_C[q+3] = bldRootRotAcc[j];
        }

        // rotate the local coordinate system from seahowl to aerodyn
        bldRootOri = bldRootOri * chrono::ChMatrix33(Q_from_AngAxis(chrono::CH_C_PI/2,chrono::VECT_Y));
       
        bldRootOri_C[i*9]   =  bldRootOri(0,0);
        bldRootOri_C[i*9+1] =  bldRootOri(0,1);
        bldRootOri_C[i*9+2] =  bldRootOri(0,2);
        bldRootOri_C[i*9+3] =  bldRootOri(1,0);
        bldRootOri_C[i*9+4] =  bldRootOri(1,1);
        bldRootOri_C[i*9+5] =  bldRootOri(1,2);
        bldRootOri_C[i*9+6] =  bldRootOri(2,0);
        bldRootOri_C[i*9+7] =  bldRootOri(2,1);
        bldRootOri_C[i*9+8] =  bldRootOri(2,2);
    }

    pImpl.SetNumBlades(nblades);
    pImpl.SetBldRootPos(bldRootPos_C);
    pImpl.SetBldRootOri(bldRootOri_C);
    pImpl.SetBldRootVel(bldRootVel_C);
    pImpl.SetBldRootAcc(bldRootAcc_C);
}

void seahowl::aero::AeroDyn::setMotionMesh(seahowl::core::Turbine& turbine) {
    auto   nblades = turbine.blades.size();
    auto   nMeshPerBlade = turbine.rotor.blades[0]->elasto->nodes.size();
    auto   nMesh = nMeshPerBlade * nblades;
    float  *meshPos_C = new float  [3 * nMesh];
    double *meshOri_C = new double [9 * nMesh];
    float  *meshVel_C = new float  [6 * nMesh];
    float  *meshAcc_C = new float  [6 * nMesh];

    for (int i = 0; i < nblades; i++) {
        for (int j = 0; j < nMeshPerBlade; j++) {
            auto meshPos     = turbine.rotor.blades[i]->elasto->nodes[j]->GetPos();
            auto meshOri     = turbine.rotor.blades[i]->elasto->nodes[j]->GetA();
            auto meshTranVel = turbine.rotor.blades[i]->elasto->nodes[j]->GetPos_dt();
            auto meshRotVel  = turbine.rotor.blades[i]->elasto->nodes[j]->GetWvel_par();
            auto meshTranAcc = turbine.rotor.blades[i]->elasto->nodes[j]->GetPos_dtdt();
            auto meshRotAcc  = turbine.rotor.blades[i]->elasto->nodes[j]->GetWacc_par();

            auto ii = i * nMeshPerBlade + j;
            for (int k = 0; k < 3; k++) {
                int p = ii*3 + k;
                int q = ii*6 + k;
                meshPos_C[p]      = meshPos[k];
                meshVel_C[q]  = meshTranVel[k];
                meshVel_C[q+3] = meshRotVel[k];
                meshAcc_C[q]  = meshTranAcc[k];
                meshAcc_C[q+3] = meshRotAcc[k];
            }

            // rotate the local coordinate system from seahowl to aerodyn
            meshOri = meshOri * chrono::ChMatrix33(Q_from_AngAxis(chrono::CH_C_PI/2,chrono::VECT_Y));
            
            meshOri_C[ii*9]   =  meshOri(0,0);
            meshOri_C[ii*9+1] =  meshOri(0,1);
            meshOri_C[ii*9+2] =  meshOri(0,2);
            meshOri_C[ii*9+3] =  meshOri(1,0);
            meshOri_C[ii*9+4] =  meshOri(1,1);
            meshOri_C[ii*9+5] =  meshOri(1,2);
            meshOri_C[ii*9+6] =  meshOri(2,0);
            meshOri_C[ii*9+7] =  meshOri(2,1);
            meshOri_C[ii*9+8] =  meshOri(2,2);

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
        std::cout << "AeroDyn/InflowWind INFO: " << ErrMsg << std::endl;
    } else if (ErrStat == 2) {
        std::cerr << "AeroDyn/InflowWind WARNING: " << ErrMsg << std::endl;
    } else {
        std::cerr << "AeroDyn/InflowWind ERROR: " << ErrMsg << std::endl; 
    }
}

void seahowl::aero::AeroDynInflowLib::SetADINFILE(std::string name)
{
    std::cout << "Set Aeodyn INFILE: '" << name << "'\n";
    ADinputFileString = name;    
    ADinputFileStringLength = ADinputFileString.length();
}

void seahowl::aero::AeroDynInflowLib::SetIFWINFILE(std::string name)
{
    std::cout << "Set InflowWind INFILE: '" << name << "'\n";
    IfWinputFileString = name;
    IfWinputFileStringLength = IfWinputFileString.length();
}

void seahowl::aero::AeroDynInflowLib::SetOUTNAME(std::string name)
{
    std::cout << "Set Output filename: '" << name << "'\n";
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

void seahowl::aero::AeroDynInflowLib::SetHubPos(float  *hubPos) {
    HubPos = hubPos;
} 

void seahowl::aero::AeroDynInflowLib::SetHubOri(double *hubOri) {
    HubOri = hubOri;
}

void seahowl::aero::AeroDynInflowLib::SetHubVel(float  *hubVel) {
    HubVel = hubVel;
}

void seahowl::aero::AeroDynInflowLib::SetHubAcc(float  *hubAcc) {
    HubAcc = hubAcc;
}

void seahowl::aero::AeroDynInflowLib::SetNacPos(float  *nacPos) {
    NacPos = nacPos;
}

void seahowl::aero::AeroDynInflowLib::SetNacOri(double *nacOri) {
    NacOri = nacOri;
}

void seahowl::aero::AeroDynInflowLib::SetNacVel(float  *nacVel) {
    NacVel = nacVel;
}

void seahowl::aero::AeroDynInflowLib::SetNacAcc(float  *nacAcc) {
    NacAcc = nacAcc;
}

void seahowl::aero::AeroDynInflowLib::SetNumBlades(int nBlades) {
    NumBlades = nBlades;
}

void seahowl::aero::AeroDynInflowLib::SetBldRootPos(float  *bldRootPos) {
    BldRootPos = bldRootPos;
}

void seahowl::aero::AeroDynInflowLib::SetBldRootOri(double *bldRootOri) {
    BldRootOri = bldRootOri;
}

void seahowl::aero::AeroDynInflowLib::SetBldRootVel(float  *bldRootVel) {
    BldRootVel = bldRootVel;
}

void seahowl::aero::AeroDynInflowLib::SetBldRootAcc(float  *bldRootAcc) {
    BldRootAcc = bldRootAcc;
}

void seahowl::aero::AeroDynInflowLib::SetNumMeshPts(int nMeshPtsAllBlades) {
    NumMeshPts = nMeshPtsAllBlades;
}

void seahowl::aero::AeroDynInflowLib::SetMeshPos(float  *meshPosAllBlades) {
    MeshPos = meshPosAllBlades;
}

void seahowl::aero::AeroDynInflowLib::SetMeshOri(double *meshOriAllBlades) {
    MeshOri = meshOriAllBlades;
}

void seahowl::aero::AeroDynInflowLib::SetMeshVel(float  *meshVelAllBlades) {
    MeshVel = meshVelAllBlades;
}

void seahowl::aero::AeroDynInflowLib::SetMeshAcc(float  *meshAccAllBlades) {
    MeshAcc = meshAccAllBlades;
}

void seahowl::aero::AeroDynInflowLib::SetAeroLoads(float *meshFrcAllBlades) {
    MeshFrc = meshFrcAllBlades;
}

void seahowl::aero::AeroDynInflowLib::Init()
{
    const char *ADinputFile = ADinputFileString.c_str();
    const char *IfWinputFile = IfWinputFileString.c_str();

    gravity      =   9.80665; // Gravitational acceleration (m/s^2)
    defFldDens   =     1.225; // Air density (kg/m^3)
    defKinVisc   = 1.464E-05; // Kinematic viscosity of working fluid (m^2/s)
    defSpdSound  =     335.0; // Speed of sound in working fluid (m/s)
    defPatm      =  103500.0; // Atmospheric pressure (Pa) [used only for an MHK turbine cavitation check]
    defPvap      =    1700.0; // Vapour pressure of working fluid (Pa) [used only for an MHK turbine cavitation check]
    WtrDpth      =       0.0; // Water depth (m)
    MSL2SWL      =       0.0; // Offset between still-water level and mean sea level (m) [positive upward]

    // Interpolation order (must be 1: linear, or 2: quadratic)
    InterpOrder  =         1; // default of linear interpolation

    // Initial time related variables
    //Time         = 0.0; // initial time
    //DT           = 0.1; // typical default for AD
    TMax         =  180;  // typical default for AD 

    // VTK
    WrVTK        = 2; //default of no vtk output
    WrVTK_Type   = 1; // defautl of surface meshes
    VTKNacDim    = new float[6]{0,-4.2751,-4.2751,12,8.552,8.552}; //default nacelle dimension for VTK surface rendering [x0,y0,z0,Lx,Ly,Lz] (m)
    VTKHubRad    = 3.97; // default hub radius for VTK surface rendering

    //NumBlades    = 3;
    //NumMeshPts   = 1;

    AeroDyn_Inflow_C_Init(
        ADinputFilePassed, 
        &ADinputFile,
        ADinputFileStringLength, 
        IfWinputFilePassed, 
        &IfWinputFile,
        IfWinputFileStringLength, 
        OutRootName,  
        gravity, 
        defFldDens, 
        defKinVisc, 
        defSpdSound,
        defPatm, 
        defPvap, 
        WtrDpth, 
        MSL2SWL,                
        InterpOrder, 
        Time, 
        DT, 
        TMax,                  
        storeHHVel, 
        TransposeDCM,                               
        WrVTK, 
        WrVTK_Type, 
        VTKNacDim, 
        VTKHubRad,        
        HubPos, 
        HubOri,                                        
        NacPos, 
        NacOri,                                        
        NumBlades, 
        BldRootPos, 
        BldRootOri,                   
        NumMeshPts, 
        MeshPos, 
        MeshOri,              
        NumChannels, 
        OutputChannelNames, 
        OutputChannelUnits, 
        ErrStat, 
        ErrMsg);

        CheckError();
}

void seahowl::aero::AeroDynInflowLib::Calcul() {

    float *MeshFrc_C = new float [6 * NumMeshPts];
    AeroDyn_Inflow_C_CalcOutput(
        Time,
        HubPos, 
        HubOri, 
        HubVel, 
        HubAcc,
        NacPos, 
        NacOri, 
        NacVel, 
        NacAcc,  
        BldRootPos, 
        BldRootOri, 
        BldRootVel, 
        BldRootAcc,
        NumMeshPts,  
        MeshPos, 
        MeshOri, 
        MeshVel, 
        MeshAcc,  
        MeshFrc_C, 
        OutputChannelValues, 
        ErrStat, 
        ErrMsg);

    SetAeroLoads(MeshFrc_C);

    CheckError();
}

void seahowl::aero::AeroDynInflowLib::Update() {
    AeroDyn_Inflow_C_UpdateStates(
        TimeLast, 
        Time, 
        HubPos, 
        HubOri, 
        HubVel, 
        HubAcc,
        NacPos, 
        NacOri, 
        NacVel, 
        NacAcc,  
        BldRootPos, 
        BldRootOri, 
        BldRootVel, 
        BldRootAcc,
        NumMeshPts,  
        MeshPos, 
        MeshOri, 
        MeshVel, 
        MeshAcc, 
        ErrStat, 
        ErrMsg);

        CheckError();
}

void seahowl::aero::AeroDynInflowLib::End() {
    AeroDyn_Inflow_C_End(ErrStat, ErrMsg);
    CheckError();

    // delete [] hubPos_C, hubOri_C, hubVel_C, hubAcc_C;
    // delete [] nacPos_C, nacOri_C, nacVel_C, nacAcc_C;
    // delete [] bldRootPos_C, bldRootOri_C, bldRootVel_C, bldRootAcc_C;
    // delete [] meshPos_C, meshOri_C, meshVel_C, meshAcc_C;
}
