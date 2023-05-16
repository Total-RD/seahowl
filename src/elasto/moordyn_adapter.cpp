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
    unsigned int nCoupledDof;
    ErrStat = MoorDyn_NCoupledDOF(moordynobj, &nCoupledDof);
    if (ErrStat != MOORDYN_SUCCESS) {
        throw std::runtime_error("Failed to get the number of coupled DOF.");
    }

    fairlead_position = Eigen::VectorXd::Zero(nCoupledDof);
    fairlead_velocity = Eigen::VectorXd::Zero(nCoupledDof);
    fairlead_load = Eigen::VectorXd::Zero(nCoupledDof);

    // auto nMeshTower = turbine.tower.nodes.size();
    // auto position = turbine.tower.nodes[0]->get_position();
    // auto velocity = turbine.tower.nodes[0]->get_velocity();
    // fairlead_position << position, position, position;
    // fairlead_velocity << velocity, velocity, velocity;

    auto nMeshBlade = turbine.rotor.blades[0]->nodes.size();
    auto position0 = turbine.rotor.blades[0]->nodes[nMeshBlade - 1]->get_position();
    auto velocity0 = turbine.rotor.blades[0]->nodes[nMeshBlade - 1]->get_velocity();
    auto position1 = turbine.rotor.blades[1]->nodes[nMeshBlade - 1]->get_position();
    auto velocity1 = turbine.rotor.blades[1]->nodes[nMeshBlade - 1]->get_velocity();
    auto position2 = turbine.rotor.blades[2]->nodes[nMeshBlade - 1]->get_position();
    auto velocity2 = turbine.rotor.blades[2]->nodes[nMeshBlade - 1]->get_velocity();
    fairlead_position << position0, position1, position2;
    fairlead_velocity << velocity0, velocity1, velocity2;
    MoorDyn_Init(moordynobj, &fairlead_position[0], &fairlead_velocity[0]);
}

void seahowl::elasto::MoorDynAdapter::step(double time, double dt, seahowl::elasto::TurbineElasto& turbine) {
    // auto nMeshTower = turbine.tower.nodes.size();
    // auto position = turbine.tower.nodes[0]->get_position() + Vector3d(time*10, 0.0, 0.0);
    // auto velocity = turbine.tower.nodes[0]->get_velocity();// + Vector3d(dt, 0.0, 0.0);
    // fairlead_position << position, position, position;
    // fairlead_velocity << velocity, velocity, velocity;

    auto nMeshBlade = turbine.rotor.blades[0]->nodes.size();
    auto position0 = turbine.rotor.blades[0]->nodes[nMeshBlade - 1]->get_position();
    auto velocity0 = turbine.rotor.blades[0]->nodes[nMeshBlade - 1]->get_velocity();
    auto position1 = turbine.rotor.blades[1]->nodes[nMeshBlade - 1]->get_position();
    auto velocity1 = turbine.rotor.blades[1]->nodes[nMeshBlade - 1]->get_velocity();
    auto position2 = turbine.rotor.blades[2]->nodes[nMeshBlade - 1]->get_position();
    auto velocity2 = turbine.rotor.blades[2]->nodes[nMeshBlade - 1]->get_velocity();
    fairlead_position << position0, position1, position2;
    fairlead_velocity << velocity0, velocity1, velocity2;

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

void seahowl::elasto::MoorDynAdapter::saveVTK(std::string filename, double time, double dt) {
    const char* infile = filename.c_str();
    MoorDyn_SaveVTK(moordynobj, infile);

    // unsigned int nMooringLine;
    // ErrStat = MoorDyn_GetNumberLines(moordynobj, &nMooringLine);
    // if (ErrStat != MOORDYN_SUCCESS) {
    //     throw std::runtime_error("Failed to get the number of lines.");
    // }

    // char filename_line[2048];
    // auto line1 = MoorDyn_GetLine(moordynobj, 1);
    // std::sprintf(filename_line, "./output/vtk/mooring_line_%03d.vtp",int(time/dt));
    // MoorDyn_SaveLineVTK(line1, filename_line);
    // for (unsigned int i = 1; i++; i <= nMooringLine) {
    //     auto line = MoorDyn_GetLine(moordynobj, i);
    //     std::sprintf(filename_line, "./output/vtk/mooring_line%d_%03d.vtm", i , int(time/dt));
    //     MoorDyn_SaveLineVTK(line, filename_line);
    // }
}
