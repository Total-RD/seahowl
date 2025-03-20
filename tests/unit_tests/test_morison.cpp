#include "fixture_components.h"

#include <seahowl/fluid/hydro/morison.h>
#include <seahowl/env/wave_models.h>
#include <seahowl/core/simulation.h>
#ifdef HAVE_HYDROCHRONO
    #include <seahowl/fluid/hydro/hydrochrono_adapter.h>
    #include <hydroc/hydro_forces.h>
#endif

#include <gtest/gtest.h>
#include <filesystem>  // C++17
using std::filesystem::path;

using namespace seahowl;

// The fixture for testing
class TestMorison : public FixtureComponents {
  protected:
    TestMorison() : FixtureComponents() {
        ref_dir /= "test_morison/ref";
        test_dir /= "test_morison/test";
    }
};

#ifdef HAVE_HYDROCHRONO
TEST_F(TestMorison, analytical_comparison) {
    double rho = 1025.0;
    auto position = seahowl::Vector3d(0.0, 0.0, -2.5);
    double duration = 30.0;
    double dt = 0.1;
    double wave_height = 5.0;
    double wave_period = 10.0;
    double water_depth = 50.0;
    double mean_water_level = 0.0;
    double diameter = 5.0;

    // create Morison coefficients
    auto coefficients = seahowl::hydro::HydroCoefficients();
    coefficients.drag_normal = 1.2;
    coefficients.drag_axial = 0.0;
    coefficients.added_mass_normal = 0.63;
    coefficients.added_mass_axial = 0.0;
    coefficients.buoyancy_factor = 0.0;

    // create Morison node 1
    auto node1 = seahowl::hydro::MorisonNode();
    node1.diameter = diameter;
    node1.coefficients = coefficients;
    node1.set_position(position);

    // create Morison node 2
    auto node2 = seahowl::hydro::MorisonNode();
    node2.diameter = diameter;
    node2.coefficients = coefficients;
    node2.set_position(position + seahowl::Vector3d(10.0, 0.0, 0.0));  // for element of length 10.0

    // create Morison element
    auto element = seahowl::hydro::MorisonElement(node1, node2);

    // create Morison node 3 (node with a given rotation)
    auto node3 = seahowl::hydro::MorisonNode();
    node3.diameter = diameter;
    node3.coefficients = coefficients;
    node3.set_position(position);
    seahowl::Quaternion rot;
    rot = seahowl::AngleAxisd(80.0 / seahowl::PI, seahowl::Vector3d::UnitX()) *
          seahowl::AngleAxisd(70.0 / seahowl::PI, seahowl::Vector3d::UnitY()) *
          seahowl::AngleAxisd(-100.0 / seahowl::PI, seahowl::Vector3d::UnitZ());
    node3.set_rotation(rot);

    // environmental conditions
    auto wave_model = std::make_shared<seahowl::env::WaveModelHydroChrono>();
    wave_model->water_depth = water_depth;
    wave_model->mean_water_level = mean_water_level;
    auto waves_hydrochrono = std::make_shared<RegularWave>();
    waves_hydrochrono->regular_wave_amplitude_ = wave_height / 2.0;
    waves_hydrochrono->regular_wave_omega_ = 2 * seahowl::PI / wave_period;
    waves_hydrochrono->mwl_ = mean_water_level;
    waves_hydrochrono->water_depth_ = water_depth;
    waves_hydrochrono->Initialize();
    wave_model->waves = waves_hydrochrono;
    // env_model
    auto env_model = seahowl::env::EnvModel();
    env_model.add_model(wave_model);

    // values to store
    auto load_analytical = seahowl::Vector3d(0.0, 0.0, 0.0);
    double time_current = 0.0;

    // add functions to record values over time for the test
    TestFrameworkDataset test_dataset({false, (ref_dir / "test_morison_analytical_comparison.csv").generic_string(),
                                       (test_dir / "test_morison_analytical_comparison.test.csv").generic_string()});
    test_dataset.test_csv.add_function("time (s)", [&time_current] { return time_current; });
    test_dataset.test_csv.add_function("fluid velocity (m/s)", [&wave_model, &time_current, &position] {
        return wave_model->get_velocity(position, time_current);
    });
    test_dataset.test_csv.add_function("fluid acceleration (m/s2)", [&wave_model, &time_current, &position] {
        return wave_model->get_acceleration(position, time_current);
    });
    test_dataset.test_csv.add_function("load node1 (N/m)", [&node1] { return node1.load; });
    test_dataset.test_csv.add_function("load node1 analytical (N/m)", [&load_analytical] { return load_analytical; });
    test_dataset.test_csv.add_function("load node2 (N/m)", [&node2] { return node2.load; });
    test_dataset.test_csv.add_function("load element (N)", [&element] { return element.get_load(); });
    test_dataset.test_csv.add_function("load node3 (N/m)", [&node2] { return node2.load; });

    while (time_current <= duration) {
        // compute loads
        node1.compute_env_loads(env_model, time_current);
        node2.compute_env_loads(env_model, time_current);
        node3.compute_env_loads(env_model, time_current);

        // compute loads with analytical formula
        auto fluid_velocity = wave_model->get_velocity(position, time_current);
        auto fluid_acceleration = wave_model->get_acceleration(position, time_current);
        load_analytical[0] =
            0.5 * rho * coefficients.drag_normal * node1.diameter * abs(fluid_velocity[0]) * fluid_velocity[0] +
            rho * (1.0 + coefficients.added_mass_normal) * PI * pow(node1.diameter, 2) / 4.0 * fluid_acceleration[0];
        load_analytical[2] =
            0.5 * rho * coefficients.drag_axial * node1.diameter * abs(fluid_velocity[2]) * fluid_velocity[2] +
            rho * (1.0 + coefficients.added_mass_axial) * PI * pow(node1.diameter, 2) / 4.0 * fluid_acceleration[2];

        // store values
        test_dataset.test_csv.write_row();
        time_current += dt;
    }
    EvaluateTest(test_dataset);
}

TEST_F(TestMorison, tower_morison) {
    // make simulation object
    auto simulation = seahowl::core::Simulation();
    simulation.dt = 0.1;
    simulation.duration = 50.0;
    simulation.outputs->dt_output = 9999.9;  // no output, using custom CSV
    simulation.outputs->has_csv = false;
    simulation.outputs->has_gui = false;
    simulation.outputs->has_vtk = false;

    double rho = 1025.0;
    double wave_height = 5.0;
    double wave_period = 10.0;
    double water_depth = 50.0;
    double mean_water_level = 0.0;

    // create Morison coefficients
    auto coefficients = seahowl::hydro::HydroCoefficients();
    coefficients.drag_normal = 1.0;
    coefficients.drag_axial = 1.0;
    coefficients.added_mass_normal = 1.0;
    coefficients.added_mass_axial = 1.0;
    coefficients.buoyancy_factor = 0.0;

    // environmental conditions
    auto wave_model = std::make_shared<seahowl::env::WaveModelHydroChrono>();
    wave_model->mean_water_level = mean_water_level;
    wave_model->water_depth = water_depth;
    auto waves_hydrochrono = std::make_shared<RegularWave>();
    waves_hydrochrono->regular_wave_amplitude_ = wave_height / 2.0;
    waves_hydrochrono->regular_wave_omega_ = 2 * seahowl::PI / wave_period;
    waves_hydrochrono->mwl_ = mean_water_level;
    waves_hydrochrono->water_depth_ = water_depth;
    waves_hydrochrono->Initialize();
    wave_model->waves = waves_hydrochrono;
    // env_model
    auto env_model = std::make_shared<seahowl::env::EnvModel>();
    env_model->add_model(wave_model);
    simulation.system_core->env_model = env_model;

    // tower
    auto tower_elasto = std::make_shared<seahowl::elasto::TowerElasto>();
    simulation.system_core->elasto.add(tower_elasto);
    auto tower_fluid = std::make_shared<seahowl::aero::TowerAero>();
    simulation.system_core->aero.add(tower_fluid);
    auto tower = std::make_shared<seahowl::core::Tower>(tower_elasto, tower_fluid);
    simulation.system_core->add(tower);
    // properties
    auto density = 7850.0;
    auto young_modulus = 2.11e11;
    auto poisson_ratio = 0.3;
    double diameter = 10.0;
    double thickness = 0.055;
    double zbottom = -50.0;  // bottom of tower
    double ztop = -5.0;      // top of tower (keep it below wave trough)
    // bottom
    auto ref1_elasto = seahowl::elasto::TowerReferencePointElasto();
    ref1_elasto.set_properties_cylinder(density, young_modulus, poisson_ratio, diameter, thickness, false);
    ref1_elasto.coordinates = seahowl::Vector3d(0.0, 0.0, zbottom);
    ref1_elasto.fraction = 0.0;
    ref1_elasto.damping_foreaft = 0.01;
    ref1_elasto.damping_sideside = 0.01;
    ref1_elasto.damping_axial = 0.01;
    ref1_elasto.damping_torsion = 0.01;
    ref1_elasto.damping_mass = 0.0;
    auto ref1_fluid = seahowl::aero::TowerReferencePointAero();
    ref1_fluid.diameter = diameter;
    ref1_fluid.coefficients = coefficients;
    ref1_fluid.coordinates = seahowl::Vector3d(0.0, 0.0, zbottom);
    ref1_fluid.fraction = 0.0;
    // top
    auto ref2_elasto = seahowl::elasto::TowerReferencePointElasto();
    ref2_elasto.set_properties_cylinder(density, young_modulus, poisson_ratio, diameter, thickness, false);
    ref2_elasto.coordinates = seahowl::Vector3d(0.0, 0.0, ztop);
    ref2_elasto.fraction = 1.0;
    ref2_elasto.damping_foreaft = 0.01;
    ref2_elasto.damping_sideside = 0.01;
    ref2_elasto.damping_axial = 0.01;
    ref2_elasto.damping_torsion = 0.01;
    ref2_elasto.damping_mass = 0.0;
    auto ref2_fluid = seahowl::aero::TowerReferencePointAero();
    ref2_fluid.diameter = diameter;
    ref2_fluid.coefficients = coefficients;
    ref2_fluid.coordinates = seahowl::Vector3d(0.0, 0.0, ztop);
    ref2_fluid.fraction = 1.0;
    //
    tower_elasto->reference_points = {ref1_elasto, ref2_elasto};
    tower_elasto->discretization_fractions = {10};
    tower_fluid->reference_points = {ref1_fluid, ref2_fluid};
    tower_fluid->discretization_fractions = {50};
    //
    tower->build();

    // fix bottom of tower
    tower_elasto->nodes.front()->set_fixed(true);

    // add functions to record values over time for the test
    TestFrameworkDataset test_dataset({false, (ref_dir / "test_morison_tower.csv").generic_string(),
                                       (test_dir / "test_morison_tower.test.csv").generic_string()});
    test_dataset.test_csv.add_function("time (s)", [&simulation] { return simulation.system_core->get_time(); });
    test_dataset.test_csv.add_function("tower base moment (N)",
                                       [&tower_elasto] { return tower_elasto->get_tower_base_moment(); });
    test_dataset.test_csv.add_function("tower base force (N)",
                                       [&tower_elasto] { return tower_elasto->get_tower_base_force(); });

    simulation.system_core->elasto.do_statics(true, 10);
    simulation.initialize();
    // store initial values
    test_dataset.test_csv.write_row();
    while (simulation.system_core->get_time() <= simulation.duration) {
        // compute loads
        simulation.step();

        // store values
        test_dataset.test_csv.write_row();
    }
    EvaluateTest(test_dataset);
}
#endif

TEST_F(TestMorison, MCF_Table) {
    double table_Y;

    // Setup TestFwDataSet
    TestFrameworkDataset test_dataset({false, (ref_dir / "test_morison_MCF_Table.csv").generic_string(),
                                       (test_dir / "test_morison_MCF_Table.test.csv").generic_string()});
    test_dataset.test_csv.add_function("Cm (-)", [&table_Y] { return table_Y; });

    seahowl::hydro::MacCamyFuchsTable mytable = seahowl::hydro::MacCamyFuchsTable();
    mytable.wave_peak_period = 10.0;

    // general options
    seahowl::hydro::HydroCoefficients coefficients;
    auto node1 = seahowl::hydro::MorisonNode();
    coefficients.use_MacCamyFuchs_correction = true;

    node1.coefficients = coefficients;

    double lambda = 1.56 * mytable.wave_peak_period * mytable.wave_peak_period;

    double DNV_table_X0 = 0.0018;
    double DNV_table_Y0 = 2.003;

    node1.diameter = 0.0018 * lambda;
    table_Y = mytable.interpolateCmBinarySearch(node1.diameter);

    test_dataset.test_csv.write_row();

    double DNV_table_X2 = 0.128;
    double DNV_table_Y2 = 2.060;
    node1.diameter = 0.128 * lambda;
    table_Y = mytable.interpolateCmBinarySearch(node1.diameter);

    test_dataset.test_csv.write_row();

    double DNV_table_X4 = 0.322;
    double DNV_table_Y4 = 1.360;
    node1.diameter = 0.322 * lambda;
    table_Y = mytable.interpolateCmBinarySearch(node1.diameter);

    test_dataset.test_csv.write_row();

    double DNV_table_X6 = 0.576;
    double DNV_table_Y6 = 0.655;
    node1.diameter = 0.576 * lambda;
    table_Y = mytable.interpolateCmBinarySearch(node1.diameter);

    test_dataset.test_csv.write_row();

    double DNV_table_X8 = 0.864;
    double DNV_table_Y8 = 0.360;
    node1.diameter = 0.864 * lambda;
    table_Y = mytable.interpolateCmBinarySearch(node1.diameter);

    test_dataset.test_csv.write_row();

    EvaluateTest(test_dataset);
}

TEST_F(TestMorison, Cd_Table) {
    double table_Cd;

    TestFrameworkDataset test_dataset({false, (ref_dir / "test_morison_Cd_Table.csv").generic_string(),
                                       (test_dir / "test_morison_Cd_Table.test.csv").generic_string()});
    test_dataset.test_csv.add_function("Cd (-)", [&table_Cd] { return table_Cd; });

    seahowl::hydro::MacCamyFuchsTable mytable = seahowl::hydro::MacCamyFuchsTable();
    mytable.wave_peak_period = 10.0;

    // general options
    seahowl::hydro::HydroCoefficients coefficients;
    auto node1 = seahowl::hydro::MorisonNode();
    coefficients.use_Cd_correction = true;

    node1.coefficients = coefficients;

    double fluid_velocity = 0.08431;
    node1.diameter = 8.1;
    table_Cd = mytable.getCd(node1.diameter, mytable.wave_peak_period,
                             fluid_velocity);  // double diameter, double wave_period, double fluid_velocity

    double CD_ref = 1.045;

    test_dataset.test_csv.write_row();
    EvaluateTest(test_dataset);
}
