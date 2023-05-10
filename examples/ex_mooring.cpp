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

#include <filesystem>  // C++17

using std::filesystem::create_directory;

#ifdef HAVE_VTK
    #include <seahowl/io/write_vtk.h>
#endif

#ifdef HAVE_IRRLICHT
    #include <chrono_irrlicht/ChVisualSystemIrrlicht.h>
    #include <seahowl/io/viz_insitu.h>
#endif

using namespace seahowl;
using namespace seahowl::elasto;
using namespace chrono;

void seabed_interaction(seahowl::elasto::SystemElasto& system, seahowl::elasto::MooringElasto& mooring) {
    auto seabed_depth = 0.0;
    auto seabed_stiffness = 1e6;
    Vector3d gravity_direction =
        system.get_gravitational_acceleration() / system.get_gravitational_acceleration().norm();
    auto seabed_position = (-gravity_direction) * seabed_depth;
    for (auto& element : mooring.elements) {
        auto element_length = dynamic_cast<seahowl::elasto::ElementMooringElasto&>(*element).get_rest_length();
        for (auto& node : element->nodes) {
            auto node_position = node->get_position();
            Vector3d node_z_vector = node_position.cwiseProduct(-gravity_direction);
            double penetration_depth = (node_z_vector - seabed_position).dot(-gravity_direction);
            if (penetration_depth < 0) {
                auto load_up = gravity_direction * seabed_stiffness * penetration_depth * mooring.diameter;
                auto load_half_element = load_up * 0.5 * element_length;
                node->set_force(node->get_force() + load_up);
            }
        }
    }
}

void setup_cables(seahowl::elasto::SystemElasto& system,
                  seahowl::elasto::MooringElasto& mooring,
                  double dt = 0.01,
                  int nsteps = 1000) {
    // check that mooring was built properly
    int nb_elements = mooring.elements.size();
    if (nb_elements != mooring.discretization_fractions.size() - 1) {
        throw std::runtime_error(
            "Number of elements and discretization fractions on mooring do not match (build mooring first?).");
    }

    // get initial and final lengths of mooring, and length increment to apply
    std::vector<double> lengths_initial(nb_elements);
    std::vector<double> lengths_final(nb_elements);
    std::vector<double> lengths_delta(nb_elements);
    for (int idx_el = 0; idx_el < nb_elements; idx_el++) {
        auto& element = dynamic_cast<seahowl::elasto::ElementMooringElasto&>(*mooring.elements[idx_el]);
        lengths_final[idx_el] =
            mooring.length * (mooring.discretization_fractions[idx_el + 1] - mooring.discretization_fractions[idx_el]);
        lengths_initial[idx_el] = (element.nodes[1]->get_position() - element.nodes[0]->get_position()).norm();
        lengths_delta[idx_el] = (lengths_final[idx_el] - lengths_initial[idx_el]) / nsteps;
    }

    // apply length increments dynamically
    for (int step = 0; step <= nsteps; step++) {
        for (int idx_el = 0; idx_el < nb_elements; idx_el++) {
            auto& element = dynamic_cast<seahowl::elasto::ElementMooringElasto&>(*mooring.elements[idx_el]);
            element.set_rest_length(lengths_initial[idx_el] + lengths_delta[idx_el] * step);
        }
        mooring.compute_hydro_loads();
        seabed_interaction(system, mooring);
        system.step(dt);
    }
}

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
    mooring.fairlead_position = Vector3d(0.0, 837.60 - 40.868, 200.0 - 14.0);
    mooring.anchor_position = Vector3d(0.0, 0.0, 0.0);
    mooring.length = 835.5;
    mooring.diameter = 0.13376;
    mooring.stiffness_axial = 753.6e6;
    mooring.density = 116.6 / (PI * pow(mooring.diameter, 2) / 4.0);
    mooring.discretization_fractions = {};
    int nelements = 40;
    for (int ii = 0; ii < nelements + 1; ii++) {
        mooring.discretization_fractions.push_back(1.0 / nelements * ii);
    }
    //
    mooring.build();
    mooring.assemble(system_elasto);

    // set bending stiffness to zero
    for (auto& element : mooring.elements) {
        dynamic_cast<ElementMooringElastoChrono&>(*element).set_bending_inertia(0);
    }

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

#ifdef HAVE_VTK
    auto output_vtk = false;
    std::vector<OutputMeshVTK> vtk_outputs;
    if (output_vtk) {
        create_directory("./output");
        auto& post_mooring = vtk_outputs.emplace_back(mooring);
        post_mooring.initialize("./output/mooring");
    }
#endif

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
    setup_cables(system_elasto, mooring, dt, 1000);
    mooring.drag_coefficient = 0.5;
    while (true) {
        time += system_chrono->GetStep();
        mooring.compute_hydro_loads();
        seabed_interaction(system_elasto, mooring);
        system_chrono->DoStepDynamics(dt);
        step += 1;
        std::cout << "time: " << time << std::endl;
#ifdef HAVE_VTK
        if (output_vtk) {
            for (auto const& vtk_output : vtk_outputs) {
                vtk_output.write(system_elasto.get_time(), step);
            }
        }
#endif
#ifdef HAVE_IRRLICHT
        application->GetDevice()->run();
        draw_system(system_chrono, application);
        application->EndScene();
#endif
    }

    return 0;
}
