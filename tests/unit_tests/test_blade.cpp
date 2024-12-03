
#include "fixture_components.h"

#include <seahowl/elasto/chrono_adapters.h>
#include <seahowl/elasto/blade_elasto.h>
#include <seahowl/io/read_json.h>

#include <gtest/gtest.h>
#include <filesystem>  // C++17
using std::filesystem::path;

using namespace seahowl;
using namespace seahowl::elasto;

// The fixture for testing
class TestBlade : public FixtureComponents {
  protected:
    TestBlade() : FixtureComponents() {
        ref_dir /= "test_blade/ref";
        test_dir /= "test_blade/test";
    }
};

TEST_F(TestBlade, mass_geometry) {
    // system
    auto system_elasto = SystemElastoChrono();
    system_elasto.set_gravitational_acceleration(Vector3d(0.0, 0.0, -9.81));

    // blade
    auto blade = seahowl::elasto::BladeElastoFEA();
    blade.reference_points = get_blade_elasto_reference_points_from_json((DATADIR / "blade.json").generic_string());
    // make 50 elements
    blade.discretization_fractions.clear();
    for (int ii = 0; ii < 51; ii++) {
        blade.discretization_fractions.push_back(0.02 * ii);
    }
    blade.build();
    blade.assemble(system_elasto);
    blade.nodes.front()->set_fixed(true);

    // Setup TestFwDataSet
    TestFrameworkDataset test_dataset(
        {false,
         (ref_dir / "test_blade_mass_geometry.values.csv").generic_string(),
         (test_dir / "test_blade_mass_geometry.values.test.csv").generic_string(),
         {"values"}});

    // check geometry
    for (int ii = 0; ii < blade.nodes.size(); ii++) {
        test_dataset.add_row_data({blade.nodes[ii]->get_position().x(), blade.nodes[ii]->get_position().y(),
                                   blade.nodes[ii]->get_position().z()});
    }

    // statics
    system_elasto.do_statics(true, 0);

    // check mass
    test_dataset.add_row_data({blade.get_mass()});

    EvaluateTest(test_dataset);
}

TEST_F(TestBlade, edgewise) {
    // Setup TestFwDataSet
    TestFrameworkDataset test_datasetvalues(
        {false,
         (ref_dir / "test_blade_edgewise.values.csv").generic_string(),
         (test_dir / "test_blade_edgewise.values.test.csv").generic_string(),
         {"values"}});

    // system
    auto system_elasto = SystemElastoChrono();
    system_elasto.set_gravitational_acceleration(Vector3d(0.0, 0.0, -9.81));

    // blade
    auto blade = seahowl::elasto::BladeElastoFEA();
    blade.fpm_mode = true;
    populate_blade_elasto_from_json((DATADIR / "blade.json").generic_string(), blade);
    // make 50 elements
    blade.discretization_fractions.clear();
    for (int ii = 0; ii < 51; ii++) {
        blade.discretization_fractions.push_back(0.02 * ii);
    }
    blade.build();
    blade.assemble(system_elasto);
    blade.nodes.front()->set_fixed(true);

    // rotate blade (flat along y axis)
    blade.rotate(PI / 2.0, Vector3d(1.0, 0.0, 0.0));

    // test deflection
    system_elasto.do_statics(true, 10);

    test_datasetvalues.add_row_data({blade.nodes.back()->get_position().z()});

    // flip blade
    blade.rotate(PI, Vector3d(0.0, 1.0, 0.0));
    system_elasto.do_statics(true, 10);

    test_datasetvalues.add_row_data({blade.nodes.back()->get_position().z()});

    EvaluateTest(test_datasetvalues);

    // static position of blade tip
    double pos0 = blade.nodes.back()->get_position().z();

    // check zero-crossings (static position of blade tip)
    int step = 0;
    double pos_y = 0.0;
    double dt = 0.02;
    int npeaks = 0;
    double natural_period = 0.0;
    double time = 0.0;
    double end_time = 10.0;
    double start_time = 0.0;
    blade.nodes.back()->set_force(Vector3d(0.0, 0.0, 1000.0), false);

    // Setup TestFwDataSet
    TestFrameworkDataset test_dataset({false,
                                       (ref_dir / "test_blade_edgewise.csv").generic_string(),
                                       (test_dir / "test_blade_edgewise.test.csv").generic_string(),
                                       {"time", "natural_period"}});

    while (time < end_time) {
        if (time > 0.5) {
            blade.nodes.back()->reset_loads();
            if (blade.nodes.back()->get_position().z() < pos0 && pos_y > pos0) {
                if (start_time == 0.0 && npeaks == 0) {
                    start_time = time;
                } else {
                    npeaks += 1;
                    natural_period = (time - start_time) / npeaks;
                    test_dataset.add_row_data({time, natural_period});
                }
            }
        }
        pos_y = blade.nodes.back()->get_position().z();
        system_elasto.step(dt);
        time += dt;
        step += 1;
    }

    EvaluateTest(test_dataset);
}

TEST_F(TestBlade, flapwise) {
    // Setup TestFwDataSet
    TestFrameworkDataset test_datasetvalues(
        {false,
         (ref_dir / "test_blade_flapwise.values.csv").generic_string(),
         (test_dir / "test_blade_flapwise.values.test.csv").generic_string(),
         {"value"}});
    // system
    auto system_elasto = SystemElastoChrono();
    system_elasto.set_gravitational_acceleration(Vector3d(0.0, 0.0, -9.81));

    // blade
    auto blade = seahowl::elasto::BladeElastoFEA();
    blade.fpm_mode = true;
    populate_blade_elasto_from_json((DATADIR / "blade.json").generic_string(), blade);
    // make 50 elements
    blade.discretization_fractions.clear();
    for (int ii = 0; ii < 51; ii++) {
        blade.discretization_fractions.push_back(0.02 * ii);
    }
    blade.build();
    blade.assemble(system_elasto);
    blade.nodes.front()->set_fixed(true);

    // rotate blade (flat along x axis)
    blade.rotate(PI / 2.0, Vector3d(0.0, 1.0, 0.0));

    // test deflection
    system_elasto.do_statics(true, 10);

    test_datasetvalues.add_row_data({blade.nodes.back()->get_position().z()});

    // flip blade
    blade.rotate(PI, Vector3d(1.0, 0.0, 0.0));
    system_elasto.do_statics(true, 10);

    test_datasetvalues.add_row_data({blade.nodes.back()->get_position().z()});

    EvaluateTest(test_datasetvalues);

    // static position of blade tip
    double pos0 = blade.nodes.back()->get_position().z();

    // check zero-crossings (static position of blade tip)
    int step = 0;
    double pos_y = 0.0;
    double dt = 0.02;
    int npeaks = 0;
    double natural_period = 0.0;
    double time = 0.0;
    double end_time = 10.0;
    double start_time = 0.0;
    blade.nodes.back()->set_force(Vector3d(0.0, 0.0, 1000.0), false);

    // Setup TestFwDataSet
    TestFrameworkDataset test_dataset({false,
                                       (ref_dir / "test_blade_flapwise.csv").generic_string(),
                                       (test_dir / "test_blade_flapwise.test.csv").generic_string(),
                                       {"time", "natural_period"}});

    while (time < end_time) {
        if (time > 0.5) {
            blade.nodes.back()->reset_loads();
            if (blade.nodes.back()->get_position().z() < pos0 && pos_y > pos0) {
                if (start_time == 0.0 && npeaks == 0) {
                    start_time = time;
                } else {
                    npeaks += 1;
                    natural_period = (time - start_time) / npeaks;
                    test_dataset.add_row_data({time, natural_period});
                }
            }
        }
        pos_y = blade.nodes.back()->get_position().z();
        system_elasto.step(dt);
        time += dt;
        step += 1;
    }

    EvaluateTest(test_dataset);
}
