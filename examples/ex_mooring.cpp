#include <seahowl/elasto/mooring_elasto.h>
#include <seahowl/elasto/chrono_adapters.h>
#include <seahowl/commons/numerics.h>

#include <chrono/physics/ChSystemSMC.h>
#include <chrono/solver/ChDirectSolverLS.h>
#include <chrono/physics/ChLinkMate.h>
#include <chrono/fea/ChNodeFEAxyzrot.h>
#include <chrono/fea/ChContactSurfaceNodeCloud.h>
#include <chrono/physics/ChBodyEasy.h>
#include <chrono/fea/ChElementCableANCF.h>
#include <chrono/fea/ChLinkPointFrame.h>

#ifdef HAVE_IRRLICHT
    #include <chrono_irrlicht/ChVisualSystemIrrlicht.h>
    #include <seahowl/io/viz_insitu.h>
#endif

using namespace seahowl;
using namespace seahowl::elasto;
using namespace chrono;

int main(int argc, char* argv[]) {
    double dt = 0.01;

    // system
    auto system_elasto = SystemElastoChrono();
    system_elasto.set_gravitational_acceleration(Vector3d(0.0, 0.0, -9.81));
    auto system_chrono = system_elasto.chobj;

    auto solver = chrono_types::make_shared<ChSolverSparseLU>();
    system_chrono->SetSolver(solver);
    solver->UseSparsityPatternLearner(true);
    solver->LockSparsityPattern(true);
    solver->SetVerbose(false);

    system_chrono->SetTimestepperType(ChTimestepper::Type::HHT);
    if (auto mystepper = std::dynamic_pointer_cast<ChTimestepperHHT>(system_chrono->GetTimestepper())) {
        mystepper->SetStepControl(false);
        mystepper->SetModifiedNewton(false);
    }

    // mooring line
    auto mooring = seahowl::elasto::MooringElasto();
    mooring.fairlead_position = Vector3d(0.0, 200.0, 200.0);
    mooring.anchor_position = Vector3d(0.0, 0.0, 0.0);
    mooring.length = 284;
    mooring.diameter = 0.13376;
    mooring.stiffness_axial = 753.6e6;
    mooring.density = 116.6 / (PI * pow(mooring.diameter, 2) / 4.0);
    mooring.discretization_fractions = {0.0, 0.1, 0.2, 0.3, 0.4, 0.5, 0.6, 0.7, 0.8, 0.9, 1.0};
    //
    mooring.build();
    mooring.assemble(system_elasto);

    // make fairlead
    auto fairlead = chrono_types::make_shared<ChBody>();
    fairlead->SetBodyFixed(true);
    system_chrono->Add(fairlead);
    // attach to fairlead
    auto fairlead_link = chrono_types::make_shared<chrono::fea::ChLinkPointFrame>();
    fairlead_link->Initialize(std::dynamic_pointer_cast<NodeElastoChronoD>(mooring.nodes.front())->chobj, fairlead);
    system_chrono->Add(fairlead_link);
    // fairlead_link->SetConstrainedCoords(true, true, true,     // x, y, z
    //                                     true, false, false);  // Rx, Ry, Rz

    // make anchor
    auto anchor = chrono_types::make_shared<ChBody>();
    anchor->SetBodyFixed(true);
    system_chrono->Add(anchor);
    // attach to anchor
    auto anchor_link = chrono_types::make_shared<chrono::fea::ChLinkPointFrame>();
    anchor_link->Initialize(std::dynamic_pointer_cast<NodeElastoChronoD>(mooring.nodes.back())->chobj, anchor);
    system_chrono->Add(anchor_link);
    // anchor_link->SetConstrainedCoords(true, true, true,     // x, y, z
    //                                   true, false, false);  // Rx, Ry, Rz

    //// assemble contact node cloud
    //// make contact material
    // auto contact_material = chrono_types::make_shared<chrono::ChMaterialSurfaceSMC>();
    // double area = chrono::CH_C_PI * pow(mooring.diameter, 2) / 4.0;
    // contact_material->SetYoungModulus(mooring.stiffness_axial / area);
    // contact_material->SetFriction(0.3f);
    // contact_material->SetRestitution(0.2f);
    // contact_material->SetAdhesion(0);
    // auto contact_cloud = chrono_types::make_shared<chrono::fea::ChContactSurfaceNodeCloud>(
    //     contact_material, std::dynamic_pointer_cast<MeshElastoChrono>(system_elasto.mesh)->chobj.get());
    // for (auto& node : mooring.nodes) {
    //     contact_cloud->AddNode(std::dynamic_pointer_cast<NodeElastoChronoD>(node)->chobj, mooring.diameter);
    // }
    // std::dynamic_pointer_cast<MeshElastoChrono>(system_elasto.mesh)->chobj->AddContactSurface(contact_cloud);
    // std::cout << "here" << std::endl;

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
    // system_chrono->Add(floor);

    double time = 0;
    double step = 0;

#ifdef HAVE_IRRLICHT
    auto application = chrono_types::make_shared<chrono::irrlicht::ChVisualSystemIrrlicht>();
    application->SetWindowTitle("SEAHOWL");
    application->Initialize();
    application->SetCameraVertical(chrono::CameraVerticalDir::Z);
    draw_system_init(system_chrono, application);
#endif
    // anchor_link->SetConstrainedCoords(false, false, false,    // x, y, z
    //                                    false, false, false);  // Rx, Ry, Rz

    // system_chrono->DoStaticRelaxing();
    // system_chrono->DoStaticLinear();
    mooring.drag_coefficient = 0;
    while (true) {
        time += system_chrono->GetStep();
        mooring.compute_hydro_loads();
        system_chrono->DoStepDynamics(dt);
        if (time < 10) {
            for (auto& element : mooring.elements) {
                auto& element_mooring = dynamic_cast<seahowl::elasto::ElementMooringElasto&>(*element);
                element_mooring.set_rest_length(element_mooring.get_rest_length() * 1.0003);
            }
        } else if (time > 15) {
            mooring.drag_coefficient = 0.5;
        }
        step += 1;
        std::cout << "time: " << time << std::endl;
#ifdef HAVE_IRRLICHT
        application->GetDevice()->run();
        draw_system(system_chrono, application);
        application->EndScene();
#endif
    }

    return 0;
}
