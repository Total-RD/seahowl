#include <seahowl/elasto/mooring_elasto.h>

#include <chrono/physics/ChSystemSMC.h>
#include <chrono/solver/ChDirectSolverLS.h>
#include <chrono/physics/ChLinkMate.h>
#include <chrono/physics/ChBodyEasy.h>

using namespace chrono;

int main(int argc, char* argv[]) {
    double dt = 0.01;

    // system
    ChSystemSMC system;
    system.Set_G_acc(ChVector<double>(0.0, 0.0, -9.81));

    auto solver = chrono_types::make_shared<ChSolverSparseLU>();
    system.SetSolver(solver);
    solver->UseSparsityPatternLearner(true);
    solver->LockSparsityPattern(true);
    solver->SetVerbose(false);

    system.SetTimestepperType(ChTimestepper::Type::HHT);
    if (auto mystepper = std::dynamic_pointer_cast<ChTimestepperHHT>(system.GetTimestepper())) {
        mystepper->SetStepControl(false);
        mystepper->SetModifiedNewton(false);
    }

    // mesh
    auto mesh = chrono_types::make_shared<chrono::fea::ChMesh>();
    system.AddMesh(mesh);

    // mooring line
    auto mooring = seahowl::elasto::MooringElasto();
    mooring.fairlead_position = chrono::ChVector<double>(0.0, 200.0, 200.0);
    mooring.anchor_position = chrono::ChVector<double>(0.0, 0.0, 0.0);
    mooring.length = 282.84;
    mooring.diameter = 0.13376;
    mooring.stiffness_axial = 753.6e6;
    mooring.density = 116.6 / (chrono::CH_C_PI * pow(mooring.diameter, 2) / 4.0);
    mooring.discretization_fractions = {0.0, 0.1, 0.2, 0.3, 0.4, 0.5, 0.6, 0.7, 0.8, 0.9, 1.0};
    //
    mooring.build();
    mooring.assemble(mesh);

    // make fairlead
    auto fairlead = chrono_types::make_shared<ChBody>();
    fairlead->SetBodyFixed(true);
    system.Add(fairlead);
    // attach to fairlead
    auto fairlead_link = chrono_types::make_shared<ChLinkMateGeneric>();
    fairlead_link->Initialize(mooring.nodes.front(), fairlead, false, mooring.nodes.front()->Frame(),
                              mooring.nodes.front()->Frame());
    system.Add(fairlead_link);
    fairlead_link->SetConstrainedCoords(true, true, true,     // x, y, z
                                        true, false, false);  // Rx, Ry, Rz

    // make anchor
    auto anchor = chrono_types::make_shared<ChBody>();
    anchor->SetBodyFixed(true);
    system.Add(anchor);
    // attach to anchor
    auto anchor_link = chrono_types::make_shared<ChLinkMateFix>();
    anchor_link->Initialize(mooring.nodes.back(), anchor);
    system.Add(anchor_link);
    anchor_link->SetConstrainedCoords(true, true, true,     // x, y, z
                                      true, false, false);  // Rx, Ry, Rz

    // // make floor
    // // material
    // auto floor_material = chrono_types::make_shared<ChMaterialSurfaceSMC>();
    // floor_material->SetYoungModulus(6e4);
    // floor_material->SetFriction(0.3);
    // floor_material->SetRestitution(0.2);
    // floor_material->SetAdhesion(0);
    // // box
    // auto floor = chrono_types::make_shared<ChBodyEasyBox>(2000, 2000, 5, 1000, true, true, floor_material);
    // floor->SetPos(ChVector<double>(0.0, 0.0, -2.7));
    // floor->SetBodyFixed(true);
    // system.Add(floor);

    double time = 0;
    double step = 0;

    // release anchor
    anchor_link->SetConstrainedCoords(false, false, false,   // x, y, z
                                      false, false, false);  // Rx, Ry, Rz

    while (true) {
        system.DoStepDynamics(dt);
        time += system.GetStep();
        step += 1;
        std::cout << time << std::endl;
        chrono::GetLog() << mooring.nodes.back()->GetPos();
    }

    return 0;
}
