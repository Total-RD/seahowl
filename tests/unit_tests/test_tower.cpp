#include "fixture_components.h"

#include <seahowl/elasto/chrono_adapters.h>
#include <seahowl/commons/numerics.h>
#include <seahowl/io/read_json.h>

#include <gtest/gtest.h>
#include <filesystem>  // C++17
using std::filesystem::path;

using namespace seahowl;
using namespace seahowl::elasto;

// The fixture for testing
class TestTower : public FixtureComponents {
  protected:
    TestTower() : FixtureComponents() {
        ref_dir /= "test_tower/ref";
        test_dir /= "test_tower/test";
    }
};

TEST_F(TestTower, mass) {
    // system
    auto system_elasto = SystemElastoChrono();

    // tower
    auto tower = seahowl::elasto::TowerElasto();
    seahowl::io::populate_tower_elasto_from_json((DATADIR / "tower.csv").generic_string(), tower);
    tower.build();
    tower.assemble(system_elasto);

    system_elasto.do_statics(true, 0);

    // Setup TestFwDataSet
    TestFrameworkDataset test_dataset({false,
                                       (ref_dir / "test_tower_mass.values.csv").generic_string(),
                                       (test_dir / "test_tower_mass.values.test.csv").generic_string(),
                                       {"values"}});

    test_dataset.add_row_data({tower.get_mass()});

    EvaluateTest(test_dataset);
}

TEST_F(TestTower, frequency) {
    // system
    auto system_elasto = SystemElastoChrono();
    system_elasto.set_gravitational_acceleration(Vector3d(0.0, 0.0, -9.81));

    // tower
    auto tower = seahowl::elasto::TowerElasto();
    seahowl::io::populate_tower_elasto_from_json((DATADIR / "tower.csv").generic_string(), tower);
    tower.discretization_fractions = {};
    tower.build();
    tower.assemble(system_elasto);
    tower.nodes.front()->set_fixed(true);

    system_elasto.do_statics(true, 0);

    // Setup TestFwDataSet
    TestFrameworkDataset test_datasetvalues({false,
                                             (ref_dir / "test_tower_frequency.values.csv").generic_string(),
                                             (test_dir / "test_tower_frequency.values.test.csv").generic_string(),
                                             {"values"}});

    // check mass
    test_datasetvalues.add_row_data({tower.get_mass()});

    EvaluateTest(test_datasetvalues);

    // static position of tower top
    double pos0 = tower.nodes.back()->get_position().x();

    // check zero-crossings (static position of tower top)
    int step = 0;
    double pos_y = 0.0;
    double dt = 0.01;
    int npeaks = 0;
    double natural_period = 0.0;
    double time = 0.0;
    double end_time = 10.0;
    double start_time = 0.0;
    tower.nodes.back()->set_force(Vector3d(100000.0, 0.0, 0.0), false);

    // Setup TestFwDataSet
    TestFrameworkDataset test_dataset({false,
                                       (ref_dir / "test_tower_frequency.csv").generic_string(),
                                       (test_dir / "test_tower_frequency.test.csv").generic_string(),
                                       {"time", "natural_period"}});

    while (time < end_time) {
        if (time > 0.5) {
            tower.nodes.back()->reset_loads();
            if (tower.nodes.back()->get_position().x() < pos0 && pos_y > pos0) {
                if (start_time == 0.0 && npeaks == 0) {
                    start_time = time;
                } else {
                    npeaks += 1;
                    natural_period = (time - start_time) / npeaks;
                    test_dataset.add_row_data({time, natural_period});
                }
            }
        }
        pos_y = tower.nodes.back()->get_position().x();
        system_elasto.step(dt);
        time += dt;
        step += 1;
    }

    EvaluateTest(test_dataset);
}

TEST_F(TestTower, cylinder_frequency) {
    // system
    auto system_elasto = SystemElastoChrono();
    system_elasto.set_gravitational_acceleration(Vector3d(0.0, 0.0, -9.81));

    // tower
    auto tower = seahowl::elasto::TowerElasto();
    // properties
    auto density = 7850.0;
    auto young_modulus = 2.11e11;
    auto poisson_ratio = 0.3;
    auto diameter = 4.0;
    auto thickness = 0.03;
    // bottom
    auto ref_point1 = seahowl::elasto::TowerReferencePointElasto();
    ref_point1.set_properties_cylinder(density, young_modulus, poisson_ratio, diameter, thickness, false);
    ref_point1.coordinates = seahowl::Vector3d(0.0, 0.0, 0.0);
    ref_point1.fraction = 0.0;
    // top
    auto ref_point2 = seahowl::elasto::TowerReferencePointElasto();
    ref_point2.set_properties_cylinder(density, young_modulus, poisson_ratio, diameter, thickness, false);
    ref_point2.coordinates = seahowl::Vector3d(0.0, 0.0, 100.0);
    ref_point2.fraction = 1.0;
    //
    tower.reference_points = {ref_point1, ref_point2};
    tower.discretization_fractions = {20};
    tower.build();
    tower.assemble(system_elasto);
    tower.nodes.front()->set_fixed(true);

    system_elasto.do_statics(true, 0);

    // Setup TestFwDataSet
    TestFrameworkDataset test_datasetvalues(
        {false,
         (ref_dir / "test_tower_cylinder_frequency.values.csv").generic_string(),
         (test_dir / "test_tower_cylinder_frequency.values.test.csv").generic_string(),
         {"values"}});

    // check mass
    test_datasetvalues.add_row_data({tower.get_mass()});

    EvaluateTest(test_datasetvalues);

    // static position of tower top
    double pos0 = tower.nodes.back()->get_position().x();

    // check zero-crossings (static position of tower top)
    int step = 0;
    double pos_y = 0.0;
    double dt = 0.01;
    int npeaks = 0;
    double natural_period = 0.0;
    double time = 0.0;
    double end_time = 10.0;
    double start_time = 0.0;
    tower.nodes.back()->set_force(Vector3d(100000.0, 0.0, 0.0), false);

    // Setup TestFwDataSet
    TestFrameworkDataset test_dataset({false,
                                       (ref_dir / "test_tower_cylinder_frequency.csv").generic_string(),
                                       (test_dir / "test_tower_cylinder_frequency.test.csv").generic_string(),
                                       {"time", "natural_period"}});

    while (time < end_time) {
        if (time > 0.5) {
            tower.nodes.back()->reset_loads();
            if (tower.nodes.back()->get_position().x() < pos0 && pos_y > pos0) {
                if (start_time == 0.0 && npeaks == 0) {
                    start_time = time;
                } else {
                    npeaks += 1;
                    natural_period = (time - start_time) / npeaks;
                    test_dataset.add_row_data({time, natural_period});
                }
            }
        }
        pos_y = tower.nodes.back()->get_position().x();
        system_elasto.step(dt);
        time += dt;
        step += 1;
    }

    EvaluateTest(test_dataset);
}

TEST_F(TestTower, conical_frequency) {
    // system
    auto system_elasto = SystemElastoChrono();
    system_elasto.set_gravitational_acceleration(Vector3d(0.0, 0.0, -9.81));

    // tower
    auto tower = seahowl::elasto::TowerElasto();
    // properties
    auto density = 7850.0;
    auto young_modulus = 2.11e11;
    auto poisson_ratio = 0.3;
    // bottom
    auto ref_point1 = seahowl::elasto::TowerReferencePointElasto();
    ref_point1.set_properties_cylinder(density, young_modulus, poisson_ratio, 4.0, 0.030, false);
    ref_point1.coordinates = seahowl::Vector3d(0.0, 0.0, 0.0);
    ref_point1.fraction = 0.0;
    // top
    auto ref_point2 = seahowl::elasto::TowerReferencePointElasto();
    ref_point2.set_properties_cylinder(density, young_modulus, poisson_ratio, 3.0, 0.015, false);
    ref_point2.coordinates = seahowl::Vector3d(0.0, 0.0, 100.0);
    ref_point2.fraction = 1.0;
    //
    tower.reference_points = {ref_point1, ref_point2};
    tower.discretization_fractions = {20};
    tower.build();
    tower.assemble(system_elasto);
    tower.nodes.front()->set_fixed(true);

    system_elasto.do_statics(true, 0);

    // Setup TestFwDataSet
    TestFrameworkDataset test_datasetvalues(
        {false,
         (ref_dir / "test_tower_conical_frequency.values.csv").generic_string(),
         (test_dir / "test_tower_conical_frequency.values.test.csv").generic_string(),
         {"values"}});

    // check mass
    test_datasetvalues.add_row_data({tower.get_mass()});

    EvaluateTest(test_datasetvalues);

    // static position of tower top
    double pos0 = tower.nodes.back()->get_position().x();

    // check zero-crossings (static position of tower top)
    int step = 0;
    double pos_y = 0.0;
    double dt = 0.01;
    int npeaks = 0;
    double natural_period = 0.0;
    double time = 0.0;
    double end_time = 10.0;
    double start_time = 0.0;
    tower.nodes.back()->set_force(Vector3d(100000.0, 0.0, 0.0), false);

    // Setup TestFwDataSet
    TestFrameworkDataset test_dataset({false,
                                       (ref_dir / "test_tower_conical_frequency.csv").generic_string(),
                                       (test_dir / "test_tower_conical_frequency.test.csv").generic_string(),
                                       {"time", "natural_period"}});

    while (time < end_time) {
        if (time > 0.5) {
            tower.nodes.back()->reset_loads();
            if (tower.nodes.back()->get_position().x() < pos0 && pos_y > pos0) {
                if (start_time == 0.0 && npeaks == 0) {
                    start_time = time;
                } else {
                    npeaks += 1;
                    natural_period = (time - start_time) / npeaks;
                    test_dataset.add_row_data({time, natural_period});
                }
            }
        }
        pos_y = tower.nodes.back()->get_position().x();
        system_elasto.step(dt);
        time += dt;
        step += 1;
    }

    EvaluateTest(test_dataset);
}
