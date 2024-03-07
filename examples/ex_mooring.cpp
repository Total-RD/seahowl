#include <seahowl/elasto/mooring_elasto.h>
#include <seahowl/elasto/chrono_adapters.h>
#include <seahowl/commons/numerics.h>
#include <seahowl/env/soil_models.h>

#include <filesystem>  // C++17
#include <spdlog/spdlog.h>

using std::filesystem::create_directory;

#ifdef HAVE_VTK
    #include <seahowl/io/write_vtk.h>
#endif

#include <seahowl/io/viz_insitu.h>
#ifdef HAVE_IRRLICHT
    #include <seahowl/io/viz_insitu_irrlicht.h>
#endif

using namespace seahowl;
using namespace seahowl::elasto;

void setup_cables(seahowl::elasto::SystemElasto& system,
                  seahowl::elasto::MooringElastoFEA& mooring,
                  seahowl::env::SoilModel& seabed,
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
        mooring.compute_seabed_loads(seabed);
        system.step(dt);
    }
}

void run_simulation() {
    double dt = 0.01;

    // system
    auto system_elasto = SystemElastoChrono();
    system_elasto.set_gravitational_acceleration(Vector3d(0.0, 0.0, -9.81));
    double fluid_density = 1000.0;

    // make fairlead
    auto fairlead = seahowl::elasto::BodyElastoChrono();
    fairlead.set_position(Vector3d(0.0, 837.60 - 58, 200.0 - 14.0));
    fairlead.set_fixed(true);
    system_elasto.add(fairlead);

    // make anchor
    auto anchor = seahowl::elasto::BodyElastoChrono();
    anchor.set_position(Vector3d(0.0, 0.0, 0.0));
    anchor.set_fixed(true);
    system_elasto.add(anchor);

    // mooring line
    auto mooring = seahowl::elasto::MooringElastoFEA(fairlead, anchor);
    mooring.length = 850.0;
    mooring.diameter = 0.333;
    mooring.stiffness_axial = 3270e6;
    mooring.stiffness_bending = 0.0;
    mooring.density_linear = 685.0;
    mooring.drag_coefficient_normal = 2.0;
    mooring.drag_coefficient_tangential = 1.15;
    mooring.added_mass_coefficient_normal = 1.0;
    mooring.added_mass_coefficient_tangential = 1.0;
    mooring.discretization_fractions = {};
    int nelements = 40;
    for (int ii = 0; ii < nelements + 1; ii++) {
        mooring.discretization_fractions.push_back(1.0 / nelements * ii);
    }
    //
    mooring.build();
    mooring.assemble(system_elasto);

    auto seabed = seahowl::env::LinearSoilModel();
    seabed.soil_position = 0.0;
    seabed.soil_normal = seahowl::Vector3d(0.0, 0.0, 1.0);
    seabed.stiffness_normal = 1e6;
    seabed.stiffness_shear = 0.0;

    double time = 0;
    double step = 0;

#ifdef HAVE_VTK
    auto output_vtk = false;
    std::vector<seahowl::io::OutputMeshVTK> vtk_outputs;
    if (output_vtk) {
        create_directory("./output");
        auto& post_mooring = vtk_outputs.emplace_back(mooring);
        post_mooring.initialize("./output/mooring");
    }
#endif

    std::unique_ptr<VisualizationInSitu> viz_insitu;
#ifdef HAVE_IRRLICHT
    viz_insitu = std::make_unique<VisualizationInSituIrrlicht>();
#else
    viz_insitu = std::make_unique<VisualizationInSity>();
#endif
    viz_insitu->initialize_elasto(system_elasto);
    viz_insitu->draw();

    spdlog::info("Setting up cables.");
    setup_cables(system_elasto, mooring, seabed, dt, 1000, fluid_density);
    spdlog::info("Cables ready.");
    while (true) {
        time += system_elasto.get_time();
        mooring.compute_hydro_loads(system_elasto.get_gravitational_acceleration(), fluid_density);
        mooring.compute_seabed_loads(seabed);
        system_elasto.step(dt);
        step += 1;
        spdlog::info("time: {}, tension {}.", time, mooring.fairlead_link->get_reaction_force().norm());
#ifdef HAVE_VTK
        if (output_vtk) {
            for (auto const& vtk_output : vtk_outputs) {
                vtk_output.write(system_elasto.get_time(), step);
            }
        }
#endif
        viz_insitu->draw();
    }
}

int main(int argc, char* argv[]) {
    try {
        run_simulation();
        return 0;
    } catch (const std::exception& e) {
        spdlog::critical(e.what());
        return 1;
    }
}
