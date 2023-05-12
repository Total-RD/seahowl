#include <seahowl/elasto/mooring_elasto.h>
#include <seahowl/elasto/chrono_adapters.h>
#include <seahowl/commons/numerics.h>

#include <filesystem>  // C++17
#include <iostream>

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
                  int nsteps = 1000,
                  double fluid_density = 1000.0) {
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
    for (size_t idx_el = 0; idx_el < nb_elements; idx_el++) {
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
        mooring.compute_hydro_loads(system.get_gravitational_acceleration(), fluid_density);
        seabed_interaction(system, mooring);
        system.step(dt);
    }
}

int main(int argc, char* argv[]) {
    double dt = 0.01;

    // system
    auto system_elasto = SystemElastoChrono();
    system_elasto.set_gravitational_acceleration(Vector3d(0.0, 0.0, -9.81));
    double fluid_density = 1000.0;

    // mooring line
    auto mooring = seahowl::elasto::MooringElasto();
    mooring.fairlead_position = Vector3d(0.0, 837.60 - 58, 200.0 - 14.0);
    mooring.anchor_position = Vector3d(0.0, 0.0, 0.0);
    mooring.length = 850.0;
    mooring.diameter = 0.33;
    mooring.stiffness_axial = 3270e6;
    mooring.density_linear = 685.0;
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
    auto fairlead = seahowl::elasto::BodyElastoChrono();
    fairlead.set_fixed(true);
    system_elasto.add(fairlead);
    // attach to fairlead
    auto fairlead_link = seahowl::elasto::LinkChronoCable();
    fairlead_link.initialize(*mooring.nodes.front(), fairlead);
    system_elasto.add(fairlead_link);
    // fairlead_link.set_constraints(true, true, true,     // x, y, z
    //                                     true, false, false);  // Rx, Ry, Rz

    // make anchor
    auto anchor = seahowl::elasto::BodyElastoChrono();
    anchor.set_fixed(true);
    system_elasto.add(anchor);
    // attach to anchor
    auto anchor_link = seahowl::elasto::LinkChronoCable();
    anchor_link.initialize(*mooring.nodes.back(), anchor);
    system_elasto.add(anchor_link);
    // anchor_link.set_constraintss(true, true, true,     // x, y, z
    //                                   true, false, false);  // Rx, Ry, Rz

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
    draw_system_init(system_elasto.chobj, application);
#endif

    mooring.drag_coefficient = 0;
    setup_cables(system_elasto, mooring, dt, 1000, fluid_density);
    mooring.drag_coefficient = 0.5;
    while (true) {
        time += system_elasto.get_time();
        mooring.compute_hydro_loads(system_elasto.get_gravitational_acceleration(), fluid_density);
        seabed_interaction(system_elasto, mooring);
        system_elasto.step(dt);
        step += 1;
        std::cout << "time: " << time << ", tension: " << fairlead_link.get_reaction_force().norm() << std::endl;
#ifdef HAVE_VTK
        if (output_vtk) {
            for (auto const& vtk_output : vtk_outputs) {
                vtk_output.write(system_elasto.get_time(), step);
            }
        }
#endif
#ifdef HAVE_IRRLICHT
        application->GetDevice()->run();
        draw_system(system_elasto.chobj, application);
        application->EndScene();
#endif
    }

    return 0;
}
