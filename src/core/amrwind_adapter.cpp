#include "seahowl/core/amrwind_adapter.h"

#include "seahowl/core/simulation.h"
#include "seahowl/core/system.h"
#include "seahowl/core/blade.h"
#include "seahowl/core/tower.h"
#include "seahowl/elasto/system_elasto.h"
#include "seahowl/elasto/blade_elasto.h"
#include "seahowl/aero/system_aero.h"
#include "seahowl/aero/blade_aero.h"
#include "seahowl/aero/tower_aero.h"
#include "seahowl/elasto/chrono_adapters.h"
#include "seahowl/io/read_json.h"
#include "seahowl/io/write_csv.h"
#include "seahowl/io/output_manager.h"
#include "seahowl/env/wind_models.h"

#include <fstream>
#include <filesystem>  // C++17
#include <nlohmann/json.hpp>
#include <spdlog/spdlog.h>
#include <spdlog/stopwatch.h>

#include <typeinfo>
#include <iostream>

using namespace seahowl::core;
namespace fs = std::filesystem;
using json = nlohmann::json;
using seahowl::Vector3d;

AmrWindAdapter::AmrWindAdapter() {
    simulation = std::make_unique<seahowl::core::Simulation>();
}

void AmrWindAdapter::initialize_from_file(const std::string& filepath) {
    spdlog::set_pattern("[%^%l%$] %v");
    spdlog::info("**************************************************************");
    spdlog::info("INITIAL AmrWindAdapter SETUP.");
    spdlog::info("**************************************************************");

    simulation->populate_from_file(filepath);
    simulation->initialize_from_config();

    auto fluid_model = std::make_shared<seahowl::env::InflowAmrWind>();
    simulation->system_core->fluid_model = fluid_model;

    for (auto& turbine : simulation->system_core->aero.turbines) {
        try {
            auto& rotor = dynamic_cast<seahowl::aero::RotorAeroBEMT&>(*turbine->rna.rotor);
            rotor.has_induction = false;
            rotor.has_tip_loss = false;
            rotor.has_hub_loss = false;
            rotor.has_tower_shadow = false;
        } catch (const std::bad_cast& e) {
            throw std::runtime_error("Need to use RotorAeroBEMT when using SEAHOWL in AMR-Wind.");
        }
    }

    // timestepping
    dt = simulation->dt;
    duration = simulation->duration;

    auto& system_core = simulation->system_core;
    auto& turbine = *system_core->turbines[0];
    auto& blade = turbine.rna.blades[0];
    auto& tower = turbine.tower;
    numBlade = turbine.rna.blades.size();
    numBladeNode = blade->aero.nodes.size();
    bladeLength = blade->aero.nodes[0].distance_from_tip;
    numTowerNode = tower.aero.nodes.size();
    towerBaseHeight = tower.aero.reference_points[0].coordinates[2];
    towerHeight = tower.aero.reference_points[numTowerNode - 1].coordinates[2];

    // Hub node + blade nodes + tower nodes
    nNodesVel = 1 +  // Hub node is the first node
                numBlade * numBladeNode + numTowerNode;

    // inflow amrwind
    auto wind_model = simulation->system_core->fluid_model;
    wind_model->wind_velocities.resize(nNodesVel, Eigen::Vector3d::Zero());
    wind_model->wind_positions.resize(nNodesVel, Eigen::Vector3d::Zero());
}

void AmrWindAdapter::init_OpFM(int* numActForcePtsBlade,
                               int* numActForcePtsTower,
                               seahowl::core::OpFM_InputType* to_cfd,
                               seahowl::core::OpFM_OutputType* from_cfd) {
    /* Motion nodes from Seahowl */
    // postion of Seahowl aero nodes
    to_cfd->pxVel_Len = nNodesVel;
    to_cfd->pyVel_Len = nNodesVel;
    to_cfd->pzVel_Len = nNodesVel;

    AllocPAry(to_cfd->pxVel, to_cfd->pxVel_Len, "pxVel");
    AllocPAry(to_cfd->pyVel, to_cfd->pyVel_Len, "pyVel");
    AllocPAry(to_cfd->pzVel, to_cfd->pzVel_Len, "pzVel");

    // velocity at Seahowl aero nodes
    from_cfd->u_Len = nNodesVel;
    from_cfd->v_Len = nNodesVel;
    from_cfd->w_Len = nNodesVel;

    AllocPAry(from_cfd->u, from_cfd->u_Len, "u");
    AllocPAry(from_cfd->v, from_cfd->v_Len, "v");
    AllocPAry(from_cfd->w, from_cfd->w_Len, "w");

    /* Actuator nodes from AMR-Wind */
    // Hub node
    nNodesForce = 1;

    // Blade nodes
    nNodesForce = nNodesForce + numBlade * *numActForcePtsBlade;

    // Tower nodes
    nNodesForce = nNodesForce + *numActForcePtsTower;

    // postion of AMR-Wind actuator nodes
    to_cfd->pxForce_Len = nNodesForce;
    to_cfd->pyForce_Len = nNodesForce;
    to_cfd->pzForce_Len = nNodesForce;

    AllocPAry(to_cfd->pxForce, to_cfd->pxForce_Len, "pxForce");
    AllocPAry(to_cfd->pyForce, to_cfd->pyForce_Len, "pyForce");
    AllocPAry(to_cfd->pzForce, to_cfd->pzForce_Len, "pzForce");

    // velocity at AMR-Wind actuator nodes
    to_cfd->xdotForce_Len = nNodesForce;
    to_cfd->ydotForce_Len = nNodesForce;
    to_cfd->zdotForce_Len = nNodesForce;

    AllocPAry(to_cfd->xdotForce, to_cfd->xdotForce_Len, "xdotForce");
    AllocPAry(to_cfd->ydotForce, to_cfd->ydotForce_Len, "ydotForce");
    AllocPAry(to_cfd->zdotForce, to_cfd->zdotForce_Len, "zdotForce");

    // Orientation matrix at AMR-Wind actuator nodes
    to_cfd->pOrientation_Len = 3 * 3 * nNodesForce;

    AllocPAry(to_cfd->pOrientation, to_cfd->pOrientation_Len, "pOrientation");

    // normalized force at AMR-Wind actuator nodes
    to_cfd->fx_Len = nNodesForce;
    to_cfd->fy_Len = nNodesForce;
    to_cfd->fz_Len = nNodesForce;

    AllocPAry(to_cfd->fx, to_cfd->fx_Len, "fx");
    AllocPAry(to_cfd->fy, to_cfd->fy_Len, "fy");
    AllocPAry(to_cfd->fz, to_cfd->fz_Len, "fz");

    // normalized moment at AMR-Wind actuator nodes
    to_cfd->momentx_Len = nNodesForce;
    to_cfd->momenty_Len = nNodesForce;
    to_cfd->momentz_Len = nNodesForce;

    AllocPAry(to_cfd->momentx, to_cfd->momentx_Len, "momentx");
    AllocPAry(to_cfd->momenty, to_cfd->momenty_Len, "momenty");
    AllocPAry(to_cfd->momentz, to_cfd->momentz_Len, "momentz");

    // chord distribution at AMR-Wind actuator nodes
    to_cfd->forceNodesChord_Len = nNodesForce;
    AllocPAry(to_cfd->forceNodesChord, to_cfd->forceNodesChord_Len, "forceNodesChord");

    // location of actuator force nodes on blade
    forceBldRnodes_Len = *numActForcePtsBlade;

    AllocPAry(forceBldRnodes, *numActForcePtsBlade, "forceBldRnodes");

    // location of actuator force nodes on tower
    forceTwrHnodes_Len = *numActForcePtsTower;

    AllocPAry(forceTwrHnodes, *numActForcePtsTower, "forceTwrHnodes");

    // mapping
    nMappings = numBlade;
    if (*numActForcePtsTower > 0) {
        nMappings = numBlade + 1;
    }

    // Create the blade and tower nodes
    CreateActForceBladeTowerNodes();

    // inflow amrwind
    auto wind_model = simulation->system_core->fluid_model;

    wind_model->u_Len = nNodesVel;
    wind_model->v_Len = nNodesVel;
    wind_model->w_Len = nNodesVel;

    AllocPAry(wind_model->u, wind_model->u_Len, "u");
    AllocPAry(wind_model->v, wind_model->v_Len, "v");
    AllocPAry(wind_model->w, wind_model->w_Len, "w");

    wind_model->pxVel_Len = nNodesVel;
    wind_model->pyVel_Len = nNodesVel;
    wind_model->pzVel_Len = nNodesVel;

    AllocPAry(wind_model->pxVel, wind_model->pxVel_Len, "pxVel");
    AllocPAry(wind_model->pyVel, wind_model->pyVel_Len, "pyVel");
    AllocPAry(wind_model->pzVel, wind_model->pzVel_Len, "pzVel");
}

void AmrWindAdapter::step(seahowl::core::OpFM_InputType* to_cfd, seahowl::core::OpFM_OutputType* from_cfd) {
    spdlog::stopwatch sw_step;
    //
    SetOpFMPositions(to_cfd, from_cfd);

    simulation->step();

    SetOpFMForces(to_cfd, from_cfd);
}

void AmrWindAdapter::AllocPAry(float*& array, int size, const std::string& name) {
    try {
        array = new float[size];
        // Initialize to zero
        for (int i = 0; i < size; i++) {
            array[i] = 0.0;
        }
    } catch (const std::bad_alloc&) {
        spdlog::error("Allocation failed for {}", name);
    }
}

void AmrWindAdapter::CreateActForceBladeTowerNodes() {
    // Blade: uniform distribution
    // To add: Non-uniform distribution may be added in the future
    auto dRforceNodes = bladeLength / (forceBldRnodes_Len - 1);
    for (int i = 0; i < forceBldRnodes_Len; i++) {
        forceBldRnodes[i] = i * dRforceNodes;
    }

    // Tower: uniform distribution
    if (nMappings > numBlade) {
        auto dRforceNodes = towerHeight / (forceTwrHnodes_Len - 1);
        for (int i = 0; i < forceTwrHnodes_Len; i++) {
            forceTwrHnodes[i] = i * dRforceNodes;
        }
    }
}

void AmrWindAdapter::InterpolateForceNodesChord(seahowl::core::OpFM_InputType* to_cfd) {
    // Hub: 1st point. This step is not necessary since forceNodesChord at hub is set to 0.0 during initialization
    to_cfd->forceNodesChord[0] = 0.0;

    int iNode = 1;
    auto turbine = simulation->system_core->aero.turbines[0];

    // Blade: we use the same discretisation as Seahowl aero, no interpolation is needed
    // To add: interpolation is needed if numActForcePtsBlade != numBladeNode
    auto blades = turbine->rna.rotor->blades;

    for (auto& blade : blades) {
        auto nodes = blade->nodes;
        for (int i = 0; i < nodes.size(); i++) {
            to_cfd->forceNodesChord[iNode] = nodes[i].properties.chord;
            iNode++;
        }
    }

    // Tower
    auto tower = turbine->tower;
    auto nodes = tower.nodes;
    for (int i = 0; i < nodes.size(); i++) {
        to_cfd->forceNodesChord[iNode] = nodes[i].diameter;
        iNode++;
    }
}

void AmrWindAdapter::CreateActForceMotionsMesh() {
    // To do list
}

// set the positions
void AmrWindAdapter::SetOpFMPositions(seahowl::core::OpFM_InputType* to_cfd, seahowl::core::OpFM_OutputType* from_cfd) {
    auto turbine = simulation->system_core->aero.turbines[0];

    // inflow amrwind
    auto wind_model = simulation->system_core->fluid_model;

    /* Hub */
    // position of seahowl node
    auto& hub = turbine->rna.rotor->body_hub;
    auto hubPos = hub.get_position();

    to_cfd->pxVel[0] = hubPos[0];
    to_cfd->pyVel[0] = hubPos[1];
    to_cfd->pzVel[0] = hubPos[2];

    // position of actuator node
    to_cfd->pxForce[0] = hubPos[0];
    to_cfd->pyForce[0] = hubPos[1];
    to_cfd->pzForce[0] = hubPos[2];

    // orientation of actuator node
    auto hubOri = hub.get_rotation().toRotationMatrix();  // get a rotation matrix 3x3
    to_cfd->pOrientation[0] = hubOri(0, 0);
    to_cfd->pOrientation[1] = hubOri(0, 1);
    to_cfd->pOrientation[2] = hubOri(0, 2);
    to_cfd->pOrientation[3] = hubOri(1, 0);
    to_cfd->pOrientation[4] = hubOri(1, 1);
    to_cfd->pOrientation[5] = hubOri(1, 2);
    to_cfd->pOrientation[6] = hubOri(2, 0);
    to_cfd->pOrientation[7] = hubOri(2, 1);
    to_cfd->pOrientation[8] = hubOri(2, 2);

    // inflow from amrwind
    wind_model->u[0] = from_cfd->u[0];
    wind_model->v[0] = from_cfd->v[0];
    wind_model->w[0] = from_cfd->w[0];

    wind_model->wind_velocities[0] = Eigen::Vector3d(from_cfd->u[0], from_cfd->v[0], from_cfd->w[0]);
    wind_model->wind_positions[0] = Eigen::Vector3d(to_cfd->pxVel[0], to_cfd->pyVel[0], to_cfd->pzVel[0]);

    /* Blade */
    int iNode = 1;

    auto blades = turbine->rna.rotor->blades;
    for (auto& blade : blades) {
        auto nodes = blade->nodes;
        for (int i = 0; i < nodes.size(); i++) {
            auto nodePos = nodes[i].get_position();

            // position of seahowl node
            to_cfd->pxVel[iNode] = nodePos[0];
            to_cfd->pyVel[iNode] = nodePos[1];
            to_cfd->pzVel[iNode] = nodePos[2];

            // position of actuator node
            to_cfd->pxForce[iNode] = nodePos[0];
            to_cfd->pyForce[iNode] = nodePos[1];
            to_cfd->pzForce[iNode] = nodePos[2];

            // velocity of actuator node
            auto nodeVel = nodes[i].get_velocity();
            to_cfd->xdotForce[iNode] = nodeVel[0];
            to_cfd->ydotForce[iNode] = nodeVel[1];
            to_cfd->zdotForce[iNode] = nodeVel[2];

            // orientation of actuator node
            auto nodeOri = nodes[i].get_rotation().toRotationMatrix();  // get a rotation matrix 3x3
            to_cfd->pOrientation[iNode * 9] = nodeOri(0, 0);
            to_cfd->pOrientation[iNode * 9 + 1] = nodeOri(0, 1);
            to_cfd->pOrientation[iNode * 9 + 2] = nodeOri(0, 2);
            to_cfd->pOrientation[iNode * 9 + 3] = nodeOri(1, 0);
            to_cfd->pOrientation[iNode * 9 + 4] = nodeOri(1, 1);
            to_cfd->pOrientation[iNode * 9 + 5] = nodeOri(1, 2);
            to_cfd->pOrientation[iNode * 9 + 6] = nodeOri(2, 0);
            to_cfd->pOrientation[iNode * 9 + 7] = nodeOri(2, 1);
            to_cfd->pOrientation[iNode * 9 + 8] = nodeOri(2, 2);

            // inflow from amrwind
            wind_model->u[iNode] = from_cfd->u[iNode];
            wind_model->v[iNode] = from_cfd->v[iNode];
            wind_model->w[iNode] = from_cfd->w[iNode];

            wind_model->wind_velocities[iNode] =
                Eigen::Vector3d(from_cfd->u[iNode], from_cfd->v[iNode], from_cfd->w[iNode]);
            wind_model->wind_positions[iNode] =
                Eigen::Vector3d(to_cfd->pxVel[iNode], to_cfd->pyVel[iNode], to_cfd->pzVel[iNode]);

            iNode++;
        }
    }

    /* Tower */
    auto tower = turbine->tower;
    auto nodes = tower.nodes;
    for (int i = 0; i < nodes.size(); i++) {
        auto nodePos = nodes[i].get_position();

        // position of seahowl node
        to_cfd->pxVel[iNode] = nodePos[0];
        to_cfd->pyVel[iNode] = nodePos[1];
        to_cfd->pzVel[iNode] = nodePos[2];

        // position of actuator node
        to_cfd->pxForce[iNode] = nodePos[0];
        to_cfd->pyForce[iNode] = nodePos[1];
        to_cfd->pzForce[iNode] = nodePos[2];

        // velocity of actuator node
        auto nodeVel = nodes[i].get_velocity();
        to_cfd->xdotForce[iNode] = nodeVel[0];
        to_cfd->ydotForce[iNode] = nodeVel[1];
        to_cfd->zdotForce[iNode] = nodeVel[2];

        // orientation of actuator node
        auto nodeOri = nodes[i].get_rotation().toRotationMatrix();  // get a rotation matrix 3x3
        to_cfd->pOrientation[iNode * 9] = nodeOri(0, 0);
        to_cfd->pOrientation[iNode * 9 + 1] = nodeOri(0, 1);
        to_cfd->pOrientation[iNode * 9 + 2] = nodeOri(0, 2);
        to_cfd->pOrientation[iNode * 9 + 3] = nodeOri(1, 0);
        to_cfd->pOrientation[iNode * 9 + 4] = nodeOri(1, 1);
        to_cfd->pOrientation[iNode * 9 + 5] = nodeOri(1, 2);
        to_cfd->pOrientation[iNode * 9 + 6] = nodeOri(2, 0);
        to_cfd->pOrientation[iNode * 9 + 7] = nodeOri(2, 1);
        to_cfd->pOrientation[iNode * 9 + 8] = nodeOri(2, 2);

        // inflow from amrwind
        wind_model->u[iNode] = from_cfd->u[iNode];
        wind_model->v[iNode] = from_cfd->v[iNode];
        wind_model->w[iNode] = from_cfd->w[iNode];

        wind_model->wind_velocities[iNode] =
            Eigen::Vector3d(from_cfd->u[iNode], from_cfd->v[iNode], from_cfd->w[iNode]);
        wind_model->wind_positions[iNode] =
            Eigen::Vector3d(to_cfd->pxVel[iNode], to_cfd->pyVel[iNode], to_cfd->pzVel[iNode]);

        iNode++;
    }

    wind_model->pxVel = to_cfd->pxVel;
    wind_model->pyVel = to_cfd->pyVel;
    wind_model->pzVel = to_cfd->pzVel;
}

// set the forces
void AmrWindAdapter::SetOpFMForces(seahowl::core::OpFM_InputType* to_cfd, seahowl::core::OpFM_OutputType* from_cfd) {
    auto turbine = simulation->system_core->aero.turbines[0];

    /* Hub */
    // position of seahowl node
    auto& hub = turbine->rna.rotor->body_hub;
    auto hubPos = hub.get_position();

    to_cfd->fx[0] = 0.0;
    to_cfd->fy[0] = 0.0;
    to_cfd->fz[0] = 0.0;

    to_cfd->momentx[0] = 0.0;
    to_cfd->momenty[0] = 0.0;
    to_cfd->momentz[0] = 0.0;

    /* Blade */
    int iNode = 1;

    auto blades = turbine->rna.rotor->blades;
    for (auto& blade : blades) {
        auto nodes = blade->nodes;
        for (int i = 0; i < nodes.size(); i++) {
            if (i == 0) {
                to_cfd->fx[iNode] = 0.5 * blade->loads[i][0];
                to_cfd->fy[iNode] = 0.5 * blade->loads[i][1];
                to_cfd->fz[iNode] = 0.5 * blade->loads[i][2];

                to_cfd->momentx[iNode] = 0.5 * blade->moments[i][0];
                to_cfd->momenty[iNode] = 0.5 * blade->moments[i][1];
                to_cfd->momentz[iNode] = 0.5 * blade->moments[i][2];
            } else if (i == (nodes.size() - 1)) {
                to_cfd->fx[iNode] = 0.5 * blade->loads[i - 1][0];
                to_cfd->fy[iNode] = 0.5 * blade->loads[i - 1][1];
                to_cfd->fz[iNode] = 0.5 * blade->loads[i - 1][2];

                to_cfd->momentx[iNode] = 0.5 * blade->moments[i - 1][0];
                to_cfd->momenty[iNode] = 0.5 * blade->moments[i - 1][1];
                to_cfd->momentz[iNode] = 0.5 * blade->moments[i - 1][2];
            } else {
                to_cfd->fx[iNode] = 0.5 * (blade->loads[i - 1][0] + blade->loads[i][0]);
                to_cfd->fy[iNode] = 0.5 * (blade->loads[i - 1][1] + blade->loads[i][1]);
                to_cfd->fz[iNode] = 0.5 * (blade->loads[i - 1][2] + blade->loads[i][2]);

                to_cfd->momentx[iNode] = 0.5 * (blade->moments[i - 1][0] + blade->moments[i][0]);
                to_cfd->momenty[iNode] = 0.5 * (blade->moments[i - 1][1] + blade->moments[i][1]);
                to_cfd->momentz[iNode] = 0.5 * (blade->moments[i - 1][2] + blade->moments[i][2]);
            }

            iNode++;
        }
    }

    /* Tower */
    auto tower = turbine->tower;
    auto nodes = tower.nodes;
    for (int i = 0; i < nodes.size(); i++) {
        to_cfd->fx[iNode] = tower.loads[i][0];
        to_cfd->fy[iNode] = tower.loads[i][1];
        to_cfd->fz[iNode] = tower.loads[i][2];

        to_cfd->momentx[iNode] = 0.0;
        to_cfd->momenty[iNode] = 0.0;
        to_cfd->momentz[iNode] = 0.0;

        iNode++;
    }
}

Vector3d seahowl::env::InflowAmrWind::get_fluid_velocity_this(const Vector3d& position, double time) const {
    Vector3d wind_velocity;

    // // position should match exactly ! to be checked
    // auto it = std::find(wind_positions.begin(), wind_positions.end(), position);
    // if (it != wind_positions.end()) {
    //     int index = std::distance(wind_positions.begin(), it);
    //     wind_velocity = wind_velocities[index];
    // } else {
    //     // throw std::runtime_error("Position not found in SEAHOWL and AMR-Wind coupling.");
    //     spdlog::warn("Position not found in SEAHOWL and AMR-Wind coupling.");
    //     std::cout << "Position is " << position.transpose() << std::endl;
    //     wind_velocity = Vector3d::Zero();
    // }

    int index = 0;
    double minDistance = (position - wind_positions[0]).norm();
    for (int i = 1; i < wind_positions.size(); ++i) {
        double distance = (position - wind_positions[i]).norm();
        if (distance < minDistance) {
            minDistance = distance;
            index = i;
        }
    }

    if (minDistance < 1.0e-2) {
        wind_velocity = wind_velocities[index];
    } else {
        spdlog::warn("Position not found in SEAHOWL and AMR-Wind coupling. MinDist: {}", minDistance);
        wind_velocity = Vector3d::Zero();
    }

    return wind_velocity;
}

Vector3d seahowl::env::InflowAmrWind::get_fluid_acceleration_this(const Vector3d& position, double time) const {
    Vector3d wind_velocity = Vector3d::Zero();
    return wind_velocity;
}
