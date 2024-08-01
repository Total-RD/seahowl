#include <seahowl/elasto/mooring_elasto.h>
#include <seahowl/hydro/mooring_hydro.h>
#include <seahowl/core/mooring.h>
#include <seahowl/elasto/chrono_adapters.h>
#include <seahowl/commons/numerics.h>
#include <seahowl/env/soil_models.h>
#include <seahowl/env/wave_models.h>

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
                  seahowl::core::Mooring& mooring,
                  seahowl::env::SoilModel& seabed,
                  seahowl::env::FluidModel& fluid,
                  double dt = 0.01,
                  int nsteps = 1000) {
    auto& mooring_elasto = mooring.elasto;

    // check that mooring was built properly
    int nb_elements = mooring_elasto.elements.size();
    if (nb_elements != mooring_elasto.discretization_fractions.size() - 1) {
        throw std::runtime_error(
            "Number of elements and discretization fractions on mooring do not match (build mooring first?).");
    }

    // get initial and final lengths of mooring, and length increment to apply
    std::vector<double> lengths_initial(nb_elements);
    std::vector<double> lengths_final(nb_elements);
    std::vector<double> lengths_delta(nb_elements);
    for (size_t idx_el = 0; idx_el < nb_elements; idx_el++) {
        auto& element = dynamic_cast<seahowl::elasto::ElementMooringElasto&>(*mooring_elasto.elements[idx_el]);
        lengths_final[idx_el] = mooring_elasto.length * (mooring_elasto.discretization_fractions[idx_el + 1] -
                                                         mooring_elasto.discretization_fractions[idx_el]);
        lengths_initial[idx_el] = (element.nodes[1]->get_position() - element.nodes[0]->get_position()).norm();
        lengths_delta[idx_el] = (lengths_final[idx_el] - lengths_initial[idx_el]) / nsteps;
    }

    // apply length increments dynamically
    for (int step = 0; step <= nsteps; step++) {
        for (int idx_el = 0; idx_el < nb_elements; idx_el++) {
            auto& element = dynamic_cast<seahowl::elasto::ElementMooringElasto&>(*mooring_elasto.elements[idx_el]);
            element.set_rest_length(lengths_initial[idx_el] + lengths_delta[idx_el] * step);
        }
        mooring.prestep(0.0, dt);
        mooring.apply_fluid_model(fluid, 0.0);
        mooring.apply_soil_model(seabed, 0.0);
        system.step(dt);
        mooring.poststep(0.0, dt);
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
    double mooring_length = 850.0;
    double mooring_diameter = 0.333;
    std::vector<double> mooring_discretization;
    int nelements = 40;
    for (int ii = 0; ii < nelements + 1; ii++) {
        mooring_discretization.push_back(1.0 / nelements * ii);
    }

    auto mooring_elasto = seahowl::elasto::MooringElastoFEA(fairlead, anchor);
    mooring_elasto.length = mooring_length;
    mooring_elasto.diameter = mooring_diameter;
    mooring_elasto.discretization_fractions = mooring_discretization;
    mooring_elasto.stiffness_axial = 3270e6;
    mooring_elasto.stiffness_bending = 0.0;
    mooring_elasto.density_linear = 685.0;

    auto mooring_hydro = seahowl::hydro::MooringHydro();
    mooring_hydro.length = mooring_length;
    mooring_hydro.diameter = mooring_diameter;
    mooring_hydro.discretization_fractions = mooring_discretization;
    mooring_hydro.coefficients.drag_normal = 2.0;
    mooring_hydro.coefficients.drag_axial = 1.15;
    mooring_hydro.coefficients.added_mass_normal = 1.0;
    mooring_hydro.coefficients.added_mass_axial = 1.0;

    auto mooring = seahowl::core::Mooring(mooring_elasto, mooring_hydro);

    mooring.build();
    mooring.elasto.assemble(system_elasto);
    system_elasto.assemble();
    mooring.initialize(0.0, 0.0);

    auto seabed = seahowl::env::LinearSoilModel();
    seabed.soil_position = 0.0;
    seabed.soil_normal = seahowl::Vector3d(0.0, 0.0, 1.0);
    seabed.stiffness_normal = 1e6;
    seabed.stiffness_shear = 0.0;

    auto fluid = seahowl::env::StillWater();
    fluid.density = 1025.0;
    fluid.mean_water_level = 1000.;
    fluid.water_depth = -1000.;

    double time = 0;
    double step = 0;

#ifdef HAVE_VTK
    auto output_vtk = false;
    std::vector<seahowl::io::OutputMeshVTK> vtk_outputs;
    if (output_vtk) {
        create_directory("./output");
        auto& post_mooring = vtk_outputs.emplace_back(mooring_elasto);
        post_mooring.initialize("./output/mooring");
    }
#endif

    std::unique_ptr<seahowl::io::VisualizationInSitu> viz_insitu;
#ifdef HAVE_IRRLICHT
    viz_insitu = std::make_unique<seahowl::io::VisualizationInSituIrrlicht>();
#else
    viz_insitu = std::make_unique<seahowl::io::VisualizationInSitu>();
#endif
    viz_insitu->initialize_elasto(system_elasto);
    viz_insitu->draw();

    spdlog::info("Setting up cables.");
    setup_cables(system_elasto, mooring, seabed, fluid, dt, 1000);
    spdlog::info("Cables ready.");
    while (true) {
        time += system_elasto.get_time();
        mooring.prestep(time, dt);
        mooring.apply_fluid_model(fluid, system_elasto.get_time());
        mooring.apply_soil_model(seabed, system_elasto.get_time());
        system_elasto.step(dt);
        mooring.poststep(time, dt);
        step += 1;
        spdlog::info("time: {}, tension {}.", time, mooring.elasto.fairlead_link->get_reaction_force().norm());
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
