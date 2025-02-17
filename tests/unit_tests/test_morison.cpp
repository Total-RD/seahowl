#include "fixture_components.h"

#include <seahowl/hydro/morison.h>
#include <seahowl/env/wave_models.h>
#include <seahowl/core/simulation.h>
#ifdef HAVE_HYDROCHRONO
    #include <seahowl/hydro/hydrochrono_adapter.h>
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
    auto wave_model = seahowl::env::WaveModelHydroChrono();
    auto waves_hydrochrono = std::make_shared<RegularWave>();
    waves_hydrochrono->regular_wave_amplitude_ = wave_height / 2.0;
    waves_hydrochrono->regular_wave_omega_ = 2 * seahowl::PI / wave_period;
    waves_hydrochrono->mwl_ = 0.0;
    waves_hydrochrono->water_depth_ = water_depth;
    waves_hydrochrono->Initialize();
    wave_model.waves = waves_hydrochrono;

    // values to store
    auto load_analytical = seahowl::Vector3d(0.0, 0.0, 0.0);
    double time_current = 0.0;

    // add functions to record values over time for the test
    TestFrameworkDataset test_dataset({false, (ref_dir / "test_morison_analytical_comparison.csv").generic_string(),
                                       (test_dir / "test_morison_analytical_comparison.test.csv").generic_string()});
    test_dataset.test_csv.add_function("time (s)", [&time_current] { return time_current; });
    test_dataset.test_csv.add_function("fluid velocity (m/s)", [&wave_model, &time_current, &position] {
        return wave_model.get_fluid_velocity(position, time_current);
    });
    test_dataset.test_csv.add_function("fluid acceleration (m/s2)", [&wave_model, &time_current, &position] {
        return wave_model.get_fluid_acceleration(position, time_current);
    });
    test_dataset.test_csv.add_function("load node1 (N/m)", [&node1] { return node1.load; });
    test_dataset.test_csv.add_function("load node1 analytical (N/m)", [&load_analytical] { return load_analytical; });
    test_dataset.test_csv.add_function("load node2 (N/m)", [&node2] { return node2.load; });
    test_dataset.test_csv.add_function("load element (N)", [&element] { return element.get_load(); });
    test_dataset.test_csv.add_function("load node3 (N/m)", [&node2] { return node2.load; });

    while (time_current <= duration) {
        // compute loads
        node1.compute_fluid_loads(wave_model, time_current);
        node2.compute_fluid_loads(wave_model, time_current);
        node3.compute_fluid_loads(wave_model, time_current);

        // compute loads with analytical formula
        auto fluid_velocity = wave_model.get_fluid_velocity(position, time_current);
        auto fluid_acceleration = wave_model.get_fluid_acceleration(position, time_current);
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
