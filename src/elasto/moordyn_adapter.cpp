#include "seahowl/elasto/moordyn_adapter.h"
#include <seahowl/elasto/turbine_elasto.h>
#include <iostream>

seahowl::elasto::MoorDynAdapter::MoorDynAdapter(std::string MoordynInfile) {
    std::cout << "Using MoorDyn in SEAHOWL" << std::endl;
    const char* infile = MoordynInfile.c_str();
    moordynobj = MoorDyn_Create(infile);
}

seahowl::elasto::MoorDynAdapter::~MoorDynAdapter() {}

void seahowl::elasto::MoorDynAdapter::initialize(seahowl::elasto::TurbineElasto& turbine) {
    unsigned int nMooringLine;
    ErrStat = MoorDyn_GetNumberLines(moordynobj, &nMooringLine);
    if (ErrStat != MOORDYN_SUCCESS) {
        throw std::runtime_error("Failed to get the number of lines.");
    }

    unsigned int nCoupledDof;
    ErrStat = MoorDyn_NCoupledDOF(moordynobj, &nCoupledDof);
    if (ErrStat != MOORDYN_SUCCESS) {
        throw std::runtime_error("Failed to get the number of coupled DOF.");
    }

    fairlead_position = Eigen::VectorXd::Zero(nCoupledDof);
    fairlead_velocity = Eigen::VectorXd::Zero(nCoupledDof);
    fairlead_load = Eigen::VectorXd::Zero(nCoupledDof);

    auto nMeshTower = turbine.tower.nodes.size();
    auto position = turbine.tower.nodes[0]->get_position();
    auto velocity = turbine.tower.nodes[0]->get_velocity();
    fairlead_position << position, position, position;
    fairlead_velocity << velocity, velocity, velocity;
    MoorDyn_Init(moordynobj, &fairlead_position[0], &fairlead_velocity[0]);
}

void seahowl::elasto::MoorDynAdapter::step(double time, double dt, seahowl::elasto::TurbineElasto& turbine) {
    auto nMeshTower = turbine.tower.nodes.size();
    auto position = turbine.tower.nodes[0]->get_position();
    auto velocity = turbine.tower.nodes[0]->get_velocity();
    fairlead_position << position, position, position;
    fairlead_velocity << velocity, velocity, velocity;

    ErrStat = MoorDyn_Step(moordynobj, &fairlead_position[0], &fairlead_velocity[0], &fairlead_load[0], &time, &dt);
    if (ErrStat != MOORDYN_SUCCESS) {
        throw std::runtime_error("Failed to compute the loads on the mooring lines.");
    }
}

// void seahowl::elasto::MoorDynAdapter::step(double time, double dt, seahowl::EntityDynamic& entity) {

//     auto position = entity.get_position();
//     auto velocity = entity.get_velocity();
//     fairlead_position << position,position,position;
//     fairlead_velocity << velocity,velocity,velocity;

//     ErrStat = MoorDyn_Step(moordynobj,&fairlead_position[0],&fairlead_velocity[0],&fairlead_load[0],&time,&dt);
//     if (ErrStat != MOORDYN_SUCCESS) {
//         throw std::runtime_error("Failed to compute the loads on the mooring lines.");
//     }
// }

void seahowl::elasto::MoorDynAdapter::end() {
    ErrStat = MoorDyn_Close(moordynobj);
    if (ErrStat != MOORDYN_SUCCESS) {
        throw std::runtime_error("Failed to release MoorDyn allocated resources.");
    }
}

void seahowl::elasto::MoorDynAdapter::saveVTK(std::string filename) {
    const char* infile = filename.c_str();
    MoorDyn_SaveVTK(moordynobj, infile);
}
