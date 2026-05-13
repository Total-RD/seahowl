// SPDX-License-Identifier: Apache-2.0
// Copyright 2022–2026 TotalEnergies
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//

#include "seahowl/fluid/hydro/hydrodyn_adapter.h"
#include "seahowl/env/seastate_adapter.h"

#include <spdlog/spdlog.h>

using namespace seahowl::fluid::hydro;

extern "C" {

void HydroDyn_C_Init(int& SeaSt_InputFilePassed,
                     const char** SeaSt_InputFileString,
                     int& SeaSt_InputFileStringLength,
                     int& HD_InputFilePassed,
                     const char** HD_InputFileString,
                     int& HD_InputFileStringLength,
                     char* OutRootName,
                     float& Gravity,
                     float& defWtrDens,
                     float& defWtrDpth,
                     float& defMSL2SWL,
                     float& PtfmRefPtPositionX,
                     float& PtfmRefPtPositionY,
                     int& NumNodePts,
                     float* InitNodePositions,
                     // int& NumWaveElev,
                     // float* WaveElevXY, //Placeholder for later
                     int& InterpOrder,
                     double& T_initial,
                     double& DT,
                     double& TMax,
                     int& NumChannels,
                     char* OutputChannelNames,
                     char* OutputChannelUnits,
                     int& ErrStat,
                     char* ErrMsg);

void HydroDyn_C_CalcOutput(double& Time,
                           int& NumNodePts,
                           float* NodePos,
                           float* NodeVel,
                           float* NodeAcc,
                           float* NodeFrc,
                           float* OutputChannelValues,
                           int& ErrStat,
                           char* ErrMsg);

void HydroDyn_C_CalcOutput_and_AddedMass(double& Time,
                                         int& NumNodePts,
                                         float* NodePos,
                                         float* NodeVel,
                                         float* NodeFrc,
                                         float* NodeAdm,
                                         float* OutputChannelValues,
                                         int& ErrStat,
                                         char* ErrMsg);

void HydroDyn_C_UpdateStates(double& Time,
                             double& TimeNext,
                             int& NumNodePts,
                             float* NodePos,
                             float* NodeVel,
                             float* NodeAcc,
                             float* OutputChannelValues,
                             int& ErrStat,
                             char* ErrMsg);

void HydroDyn_C_End(int& ErrStat, char* ErrMsg);
}

/**
 * @brief Interface to HydroDyn library.
 * Additional notes and information on the interfacing is included
 * there. Many notes taken from HydroDyn Python-C interface library.
 *
 *   Note on angles:
 *       All angles passed in are assumed to be given in radians as an Euler
 *       angle sequence R(z)*R(y)*R(x) (notice the order as this important when
 *       passing angles in). Written in matrix form as (doxygen formatted):
 *
 *          \f{eqnarray*}{
 *          M & = & R(\theta_z) R(\theta_y) R(\theta_x) \\
 *            & = & \begin{bmatrix}  \cos(\theta_z) & \sin(\theta_z) & 0 \\
 *                                  -\sin(\theta_z) & \cos(\theta_z) & 0 \\
 *                                    0      &  0      & 1 \end{bmatrix}
 *                  \begin{bmatrix}  \cos(\theta_y) & 0 & -\sin(\theta_y) \\
 *                                         0 & 1 & 0        \\
 *                                   \sin(\theta_y) & 0 & \cos(\theta_y)  \end{bmatrix}
 *                  \begin{bmatrix}   1 &  0       & 0       \\
 *                                    0 &  \cos(\theta_x) & \sin(\theta_x) \\
 *                                    0 & -\sin(\theta_x) & \cos(\theta_x) \end{bmatrix} \\
 *            & = & \begin{bmatrix}
 *             \cos(\theta_y)\cos(\theta_z) &   \cos(\theta_x)\sin(\theta_z)+\sin(\theta_x)\sin(\theta_y)\cos(\theta_z)
 * &
 *                                              \sin(\theta_x)\sin(\theta_z)-\cos(\theta_x)\sin(\theta_y)\cos(\theta_z)
 * \\
 *             -\cos(\theta_y)\sin(\theta_z)  & \cos(\theta_x)\cos(\theta_z)-\sin(\theta_x)\sin(\theta_y)\sin(\theta_z)
 * &
 *                                              \sin(\theta_x)\cos(\theta_z)+\cos(\theta_x)\sin(\theta_y)\sin(\theta_z)
 * \\
 *             \sin(\theta_y)                & -\sin(\theta_x)\cos(\theta_y) & \cos(\theta_x)\cos(\theta_y) \\
 *                  \end{bmatrix}
 *          \f}
 *
 *       When passed into the Fortran library, this Euler angle set is converted
 *       into a DCM (direction cosine matrix) and stored on the input mesh.  All
 *       calculations internally in HD are then performed using the DCM form of
 *       the input angles.
 *
 *       It should be noted that a small angle assumption when returning the
 *       outputs for the platform roll, pitch, and yaw.  These angles are
 *       assumed to be small enough that treating them as independent angles
 *       does not introduce significant error in the output channels.  This may
 *       yield a small discrepency between the values passed in and the returned
 *       output channel values.  These output channels should not be directly
 *       used -- they are only for reporting purposes.
 */

struct seahowl::fluid::hydro::HydroDynLib {
    ~HydroDynLib();

    void set_hydrodyn_infile(const std::string& name);
    void set_seastate_infile(const std::string& name);
    void set_outfile_name(const std::string& name);

    void initialize_arrays(int NumNodePts);
    void set_time(double time);

    void CheckError();
    void Init();
    void Calcul();
    void Update();
    void End();

    /*  OutRootName
     *  If HD writes a file (echo, summary, or other),
     *  use this for the root of the file name.
     */
    char OutRootName[1024];

    // Input file handling
    int SeaSt_InputFilePassed = 0;  // 1: pass the input file content; 0: pass the input file name
    int HD_InputFilePassed = 0;     // 1: pass the input file content; 0: pass the input file name

    // Input file string
    std::string HDinputFileString = "";
    std::string SSinputFileString = "";

    // Input file string length
    int HDinputFileStringLength = 0;
    int SSinputFileStringLength = 0;

    // Initial environmental conditions
    float gravity = 9.80665;   // Gravitational acceleration (m/s^2)
    float defWtrDens = 1025.;  // Water density (kg/m^3)
    float defWtrDpth = 200.;   // Water depth (m)
    float defMSL2SWL = 0.;     // Offset between still-water level and mean sea level (m) [positive upward]

    /* Number of bodies and initial reference point
     * The initial position is only set as (X,Y).  The Z value and
     * orientation is set by HD and will be returned along with the full
     * set of numBodies where it is expecting loads inputs.
     */
    float PtfmRefPtPositionX = 0.;
    float PtfmRefPtPositionY = 0.;

    /* Nodes
     * The number of nodes must be constant throughout simulation.  The
     * initial position is given in the initNodePos array (resize as
     * needed, should be Nx6).
     * Rotations are given in radians assuming small angles.  See note at
     * top of this file.
     */
    int NumNodePts = 1;  // Single ptfm attachment point for floating rigid
    float* NodePos;
    float* NodeVel;
    float* NodeAcc;
    float* NodeFrc;
    float* NodeAdm;

    int InterpOrder = 1;   // default of linear interpolation
    double Time = 0.;      // current time
    double TimeNext = 0.;  // next time
    double DT = 0.1;       // typical default for HD
    double TMax = 600.0;   // typical default for HD waves FFT

    int NumChannels = 0;  // Number of channels returned
    char OutputChannelNames[20 * 8000];
    char OutputChannelUnits[20 * 8000];
    float OutputChannelValues[20];
    int ErrStat = 0;
    char ErrMsg[1024];
};

HydroDynLib::~HydroDynLib() {
    delete[] NodePos, delete[] NodeVel, delete[] NodeAcc, delete[] NodeFrc, delete[] NodeAdm;
    End();
}

void HydroDynLib::initialize_arrays(int NumNodePts) {
    this->NumNodePts = NumNodePts;

    NodePos = new float[6 * NumNodePts]{0.0};
    NodeVel = new float[6 * NumNodePts]{0.0};
    NodeAcc = new float[6 * NumNodePts]{0.0};
    NodeFrc = new float[6 * NumNodePts]{0.0};

    NodeAdm = new float[6 * NumNodePts * 6 * NumNodePts]{0.0};
}

void HydroDynLib::set_hydrodyn_infile(const std::string& name) {
    spdlog::debug("Set HydroDyn INFILE: {}.", name);
    HDinputFileString = name;
    HDinputFileStringLength = HDinputFileString.length();
}

void HydroDynLib::set_seastate_infile(const std::string& name) {
    spdlog::debug("Set SeaState INFILE: {}.", name);
    SSinputFileString = name;
    SSinputFileStringLength = SSinputFileString.length();
}

void HydroDynLib::set_outfile_name(const std::string& name) {
    spdlog::debug("Set Hydrodyn output file: {}.", name);
    strcpy(OutRootName, name.c_str());
}

void HydroDynLib::set_time(double time) {
    Time = time;
    TimeNext = Time + DT;
}

void HydroDynLib::CheckError() {
    if (ErrStat == 0) {
        return;
    } else if (ErrStat == 1) {
        spdlog::info("HydroDyn INFO: {}.", ErrMsg);
    } else if (ErrStat == 2) {
        spdlog::warn("HydroDyn WARNING: {}", ErrMsg);
    } else {
        throw std::runtime_error("HydroDyn ERROR: " + std::string(ErrMsg));
    }
}

void HydroDynLib::Init() {
    // input files
    const char* HDinputFile = HDinputFileString.c_str();
    const char* SSinputFile = SSinputFileString.c_str();

    HydroDyn_C_Init(SeaSt_InputFilePassed, &SSinputFile, SSinputFileStringLength, HD_InputFilePassed, &HDinputFile,
                    HDinputFileStringLength, OutRootName, gravity, defWtrDens, defWtrDpth, defMSL2SWL,
                    PtfmRefPtPositionX, PtfmRefPtPositionY, NumNodePts, NodePos, InterpOrder, Time, DT, TMax,
                    NumChannels, OutputChannelNames, OutputChannelUnits, ErrStat, ErrMsg);
    CheckError();
}

void HydroDynLib::Calcul() {
    // HydroDyn_C_CalcOutput(Time, NumNodePts, NodePos, NodeVel, NodeAcc, NodeFrc, OutputChannelValues, ErrStat,
    // ErrMsg);
    HydroDyn_C_CalcOutput_and_AddedMass(Time, NumNodePts, NodePos, NodeVel, NodeFrc, NodeAdm, OutputChannelValues,
                                        ErrStat, ErrMsg);

    CheckError();
}

void HydroDynLib::Update() {
    HydroDyn_C_UpdateStates(Time, TimeNext, NumNodePts, NodePos, NodeVel, NodeAcc, OutputChannelValues, ErrStat,
                            ErrMsg);
    CheckError();
}

void HydroDynLib::End() {
    HydroDyn_C_End(ErrStat, ErrMsg);
    CheckError();
}

HydroDynAdapter::HydroDynAdapter() {
    spdlog::debug("Initialising HydroDyn Adapter");
    interface_hydrodyn = std::make_unique<HydroDynLib>();
    interface_hydrodyn->SeaSt_InputFilePassed = 0;
    interface_hydrodyn->HD_InputFilePassed = 0;
    interface_hydrodyn->set_outfile_name("Floater");
}

HydroDynAdapter::~HydroDynAdapter() {}

void HydroDynAdapter::set_hydrodyn_infile(const std::string& hydrodyn_infile) {
    interface_hydrodyn->set_hydrodyn_infile(hydrodyn_infile);
}

void HydroDynAdapter::set_seastate_infile(const std::string& seastate_infile) {
    interface_hydrodyn->set_seastate_infile(seastate_infile);
}

void HydroDynAdapter::initialize(double time, double dt, const std::vector<EntityDynamic*>& nodes) {
    spdlog::info("Initialising HydroDyn from SEAHOWL");

    interface_hydrodyn->DT = dt;
    interface_hydrodyn->set_time(time);

    // get the number of body
    int NumNodePts = nodes.size();

    interface_hydrodyn->initialize_arrays(NumNodePts);

    // resize vector of hydrodyn loads and moments
    forces_hydrodyn.resize(NumNodePts, Vector3d(0.0, 0.0, 0.0));
    moments_hydrodyn.resize(NumNodePts, Vector3d(0.0, 0.0, 0.0));
    added_mass_matrix.setZero(6 * NumNodePts, 6 * NumNodePts);

    // update turbine variables
    update_nodes_motion(nodes);

    interface_hydrodyn->Init();
}

void HydroDynAdapter::update_nodes_motion(const std::vector<EntityDynamic*>& nodes) {
    int NumNodePts = nodes.size();

    for (int i = 0; i < NumNodePts; i++) {
        auto& node = *nodes[i];
        auto node_pos = node.get_position();
        auto node_rot = node.get_rpy_angles();
        auto node_vel = node.get_velocity();
        auto node_rotvel = node.get_rotational_velocity(false);  // in global frame
        auto node_acc = node.get_acceleration();
        auto node_rotacc = node.get_rotational_acceleration(false);  // in global frame

        for (int j = 0; j < 3; j++) {
            int ii = i * 6 + j;
            interface_hydrodyn->NodePos[ii] = node_pos[j];
            interface_hydrodyn->NodePos[ii + 3] = node_rot[j];
            interface_hydrodyn->NodeVel[ii] = node_vel[j];
            interface_hydrodyn->NodeVel[ii + 3] = node_rotvel[j];
            interface_hydrodyn->NodeAcc[ii] = node_acc[j];
            interface_hydrodyn->NodeAcc[ii + 3] = node_rotacc[j];
        }
    }
}

void HydroDynAdapter::setup_environment(const env::EnvModel& env_model) {
    if (interface_hydrodyn->SSinputFileStringLength > 0) {
        spdlog::info("HydroDynAdapter: SeaState file was already passed (\"{}\"), ignoring setup from environment.",
                     interface_hydrodyn->SSinputFileString);
        return;
    }
    std::shared_ptr<env::SeaStateAdapter> seastate_adapter;
    bool found_seastate_wave_model = false;
    for (auto fluid_model : env_model.fluid_models.get_models()) {
        if (std::shared_ptr<env::SeaStateAdapter> model =
                std::dynamic_pointer_cast<env::SeaStateAdapter>(fluid_model)) {
            if (!found_seastate_wave_model) {
                found_seastate_wave_model = true;
                seastate_adapter = model;
            } else {
                throw std::runtime_error("HydroDyn adapter can only handle one SeaState model at a time.");
            }
        }
    }
    if (!found_seastate_wave_model) {
        throw std::runtime_error("HydroDyn adapter requires an SeaState model to be set up in the environment.");
    }
    std::string seastate_infile = seastate_adapter->get_seastate_infile();
    set_seastate_infile(seastate_infile);
}

void HydroDynAdapter::compute_loads(double time, const std::vector<EntityDynamic*>& nodes) {
    interface_hydrodyn->set_time(time);
    update_nodes_motion(nodes);
    interface_hydrodyn->Update();
    interface_hydrodyn->Calcul();

    int nb_nodes = interface_hydrodyn->NumNodePts;

    // get loads from HydroDyn
    for (int ii = 0; ii < nb_nodes; ii++) {
        forces_hydrodyn[ii][0] = interface_hydrodyn->NodeFrc[ii * 6 + 0];
        forces_hydrodyn[ii][1] = interface_hydrodyn->NodeFrc[ii * 6 + 1];
        forces_hydrodyn[ii][2] = interface_hydrodyn->NodeFrc[ii * 6 + 2];
        moments_hydrodyn[ii][0] = interface_hydrodyn->NodeFrc[ii * 6 + 3];
        moments_hydrodyn[ii][1] = interface_hydrodyn->NodeFrc[ii * 6 + 4];
        moments_hydrodyn[ii][2] = interface_hydrodyn->NodeFrc[ii * 6 + 5];
    }

    // get added mass matrix from HydroDyn
    for (int jj = 0; jj < 6 * nb_nodes * 6 * nb_nodes; jj++) {
        added_mass_matrix(jj / (6 * nb_nodes), jj % (6 * nb_nodes)) = interface_hydrodyn->NodeAdm[jj];
    }
}

void HydroDynAdapter::end() {
    interface_hydrodyn->End();
}

FloaterHydroDyn::FloaterHydroDyn(const std::string& hydrodyn_filepath) {
    hydrodyn = std::make_unique<HydroDynAdapter>();
    hydrodyn->set_hydrodyn_infile(hydrodyn_filepath);
}

void FloaterHydroDyn::set_seastate_infile(const std::string& seastate_infile) {
    hydrodyn->set_seastate_infile(seastate_infile);
}

void FloaterHydroDyn::setup_environment(const env::EnvModel& env_model) {
    hydrodyn->setup_environment(env_model);
}

void FloaterHydroDyn::initialize(double time, double dt) {
    std::vector<EntityDynamic*> bodies;
    bodies.push_back(body_main.get());
    hydrodyn->initialize(time, dt, bodies);
}

void FloaterHydroDyn::compute_env_loads(const env::EnvModel& env_model, double time) {
    FloaterHydro::compute_env_loads(env_model, time);

    std::vector<EntityDynamic*> bodies;
    bodies.push_back(body_main.get());
    hydrodyn->compute_loads(time, bodies);

    // set floater forces and added mass (to be called from seahowl::core::Floater and passed to elasto)
    force_hydro = hydrodyn->forces_hydrodyn[0];
    torque_hydro = hydrodyn->moments_hydrodyn[0];
    added_mass_matrix = hydrodyn->added_mass_matrix;

    auto& body = bodies[0];

    // express added mass matrix from global to local
    Eigen::Matrix<double, 6, 6> rot66 = Eigen::Matrix<double, 6, 6>::Zero();
    Eigen::Matrix<double, 3, 3> rot33(body->get_rotation_matrix());
    Eigen::Matrix<double, 3, 3> rotI = Eigen::Matrix<double, 3, 3>::Identity();
    rot66.block<3, 3>(0, 0) = rot33.block(0, 0, 3, 3);
    rot66.block<3, 3>(3, 3) = rot33.block(0, 0, 3, 3);
    added_mass_matrix = rot66.inverse() * added_mass_matrix;
}

MonopileHydroDyn::MonopileHydroDyn(const std::string& hydrodyn_filepath) {
    hydrodyn = std::make_unique<HydroDynAdapter>();
    hydrodyn->set_hydrodyn_infile(hydrodyn_filepath);
    has_nodal_distributed_loads = false;
}

void MonopileHydroDyn::set_seastate_infile(const std::string& seastate_infile) {
    hydrodyn->set_seastate_infile(seastate_infile);
}

void MonopileHydroDyn::setup_environment(const env::EnvModel& env_model) {
    hydrodyn->setup_environment(env_model);
}

void MonopileHydroDyn::initialize(double time, double dt) {
    MonopileHydro::initialize(time, dt);

    std::vector<EntityDynamic*> nodes_hd;
    for (auto& node : nodes) {
        nodes_hd.push_back(&node);
    }
    hydrodyn->initialize(time, dt, nodes_hd);
}

void MonopileHydroDyn::compute_env_loads(const env::EnvModel& env_model, double time) {
    // call HydroDyn
    std::vector<EntityDynamic*> nodes_hd;
    for (auto& node : nodes) {
        nodes_hd.push_back(&node);
    }
    hydrodyn->compute_loads(time, nodes_hd);

    for (int ii = 0; ii < nodes.size(); ii++) {
        auto& node = dynamic_cast<seahowl::fluid::hydro::MorisonNode&>(nodes[ii]);

        // loads
        node.load = hydrodyn->forces_hydrodyn[ii];
        node.load_noacc = hydrodyn->forces_hydrodyn[ii];

        // added mass matrix
        for (int row = 0; row < 6; row++) {
            for (int col = 0; col < 6; col++) {
                node.added_mass_matrix(row, col) = hydrodyn->added_mass_matrix(row + ii * 6, col + ii * 6);
            }
        }

        // express added mass matrix from global to local
        Eigen::Matrix<double, 6, 6> rot66 = Eigen::Matrix<double, 6, 6>::Zero();
        Eigen::Matrix<double, 3, 3> rot33(node.get_rotation_matrix());
        Eigen::Matrix<double, 3, 3> rotI = Eigen::Matrix<double, 3, 3>::Identity();
        rot66.block<3, 3>(0, 0) = rot33.block(0, 0, 3, 3);
        rot66.block<3, 3>(3, 3) = rot33.block(0, 0, 3, 3);

        node.added_mass_matrix = rot66.inverse() * node.added_mass_matrix;
    }
}
