#include <seahowl/elasto/mooring_elasto.h>
#include <seahowl/hydro/mooring_hydro.h>
#include <seahowl/core/mooring.h>
#include <seahowl/core/simulation.h>
#include <seahowl/core/system.h>
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

void run_simulation() {
    seahowl::set_log_level_global("debug");

    auto simulation = seahowl::core::Simulation();
    simulation.dt = 0.01;
    simulation.outputs->has_gui = true;

    double mean_water_level = 0.0;
    double water_depth = 200.0;

    // system
    auto& system_core = simulation.system_core;

    // fluid model
    auto fluid_model = std::make_shared<seahowl::env::StillWater>();
    system_core->fluid_model = fluid_model;
    fluid_model->density = 1025.0;
    fluid_model->mean_water_level = mean_water_level;
    fluid_model->water_depth = water_depth;

    // soil model
    auto soil_model = std::make_shared<seahowl::env::LinearSoilModel>();
    system_core->soil_model = soil_model;
    soil_model->soil_position = mean_water_level - water_depth;
    soil_model->soil_normal = seahowl::Vector3d(0.0, 0.0, 1.0);
    soil_model->stiffness_normal = 1e6;
    soil_model->stiffness_shear = 0.0;

    // make bodies to attach mooring line (fairlead + anchor)

    // make fairlead
    auto fairlead = seahowl::elasto::BodyElastoChrono();
    system_core->elasto.add(fairlead);
    fairlead.set_position(Vector3d(0.0, 0.0, mean_water_level - 14.0));
    fairlead.set_fixed(true);

    // make anchor
    auto anchor = seahowl::elasto::BodyElastoChrono();
    system_core->elasto.add(anchor);
    anchor.set_position(Vector3d(0.0, 58.0 - 837.60, mean_water_level - water_depth));
    anchor.set_fixed(true);

    // create mooring line

    // elasto
    auto mooring_elasto = std::make_shared<seahowl::elasto::MooringElastoFEA>(fairlead, anchor);
    system_core->elasto.add(mooring_elasto);
    // hydro
    auto mooring_hydro = std::make_shared<seahowl::hydro::MooringHydro>();
    system_core->aero.add(mooring_hydro);
    // core
    auto mooring = std::make_shared<seahowl::core::Mooring>(*mooring_elasto, *mooring_hydro);
    system_core->add(mooring);

    // set mooring properties
    int nelements = 40;
    std::vector<double> discretization_fractions;
    for (int ii = 0; ii < nelements + 1; ii++) {
        discretization_fractions.push_back(1.0 / nelements * ii);
    }
    // elasto
    mooring->elasto.stiffness_axial = 3270e6;
    mooring->elasto.stiffness_bending = 0.0;
    mooring->elasto.density_linear = 685.0;
    mooring->elasto.discretization_fractions = discretization_fractions;
    // hydro
    mooring->hydro.coefficients.drag_normal = 2.0;
    mooring->hydro.coefficients.drag_axial = 1.15;
    mooring->hydro.coefficients.added_mass_normal = 1.0;
    mooring->hydro.coefficients.added_mass_axial = 1.0;
    mooring->hydro.discretization_fractions = discretization_fractions;
    // core (elasto + hydro)
    mooring->set_length(850.0);
    mooring->set_diameter(0.333);

    system_core->build();
    simulation.initialize();
    system_core->run_presimulation(100.0, 0.01, true, true);

    while (true) {
        simulation.step();
        spdlog::info("Tension at fairlead: {}.", mooring->elasto.fairlead_link->get_reaction_force().norm());
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
