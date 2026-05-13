// SPDX-License-Identifier: Apache-2.0
// Copyright 2022–2026 TotalEnergies
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//

// Local test headers
#include "fixture_components.h"

// SEAHOWL headers
#include <seahowl/elasto.h>
#include <seahowl/io.h>

// Third-party libraries
#include <gtest/gtest.h>

// Standard library
#include <filesystem>

using std::filesystem::path;

using namespace seahowl;
using namespace seahowl::elasto;

// The fixture for testing
class TestTower : public FixtureComponents {
  protected:
    TestTower() : FixtureComponents() {
        root_dir /= "test_tower";
        ref_dir /= "test_tower/ref";
        test_dir /= "test_tower/test";
    }
};

TEST_F(TestTower, mass) {
    // system
    auto system_elasto = SystemElastoChrono();

    // tower
    auto tower = seahowl::elasto::TowerElasto();
    seahowl::io::populate_tower_elasto_from_file((DATADIR / "IEA15MW/onshore/tower.csv").generic_string(), tower);
    tower.build();
    tower.assemble(system_elasto);

    system_elasto.do_statics(true, 0);

    // test mass
    TestFrameworkDataset test_dataset_mass({false, (ref_dir / "test_tower_mass.csv").generic_string(),
                                            (test_dir / "test_tower_mass.test.csv").generic_string()});
    test_dataset_mass.test_csv.add_function("mass [kg]", [&tower] { return tower.get_mass(); });
    test_dataset_mass.test_csv.write_row();
    EvaluateTest(test_dataset_mass);
}

TEST_F(TestTower, frequency) {
    // system
    auto system_elasto = SystemElastoChrono();
    system_elasto.set_gravitational_acceleration(Vector3d(0.0, 0.0, -9.81));

    // tower
    auto tower = seahowl::elasto::TowerElasto();
    seahowl::io::populate_tower_elasto_from_file((DATADIR / "IEA15MW/onshore/tower.csv").generic_string(), tower);
    tower.discretization_fractions = {};
    tower.build();
    tower.assemble(system_elasto);
    tower.nodes.front()->set_fixed(true);

    // Setup TestFwDataSet
    TestFrameworkDataset test_dataset({false, (ref_dir / "test_tower_frequency.csv").generic_string(),
                                       (test_dir / "test_tower_frequency.test.csv").generic_string()});
    test_dataset.test_csv.add_function("time [s]", [&system_elasto] { return system_elasto.get_time(); });
    test_dataset.test_csv.add_function("top x [m]", [&tower] { return tower.nodes.back()->get_position().x(); });

    // decay
    double dt = 0.01;
    double duration = 10.0;
    tower.nodes.back()->set_force(Vector3d(-3000.0, 0.0, 0.0), false);
    system_elasto.do_statics(true, 10);
    tower.nodes.back()->set_force(Vector3d(0.0, 0.0, 0.0), false);
    while (system_elasto.get_time() <= duration) {
        test_dataset.test_csv.write_row();
        system_elasto.step(dt);
    }

    EvaluateTest(test_dataset);
}

TEST_F(TestTower, frequency_json) {
    // system
    auto system_elasto = SystemElastoChrono();
    system_elasto.set_gravitational_acceleration(Vector3d(0.0, 0.0, -9.81));

    // tower
    auto tower = seahowl::elasto::TowerElasto();
    seahowl::io::populate_tower_elasto_from_file((DATADIR / "IEA15MW/onshore/tower.json").generic_string(), tower);
    tower.discretization_fractions = {};
    tower.build();
    tower.assemble(system_elasto);
    tower.nodes.front()->set_fixed(true);

    // Setup TestFwDataSet
    TestFrameworkDataset test_dataset({false, (ref_dir / "test_tower_frequency_json.csv").generic_string(),
                                       (test_dir / "test_tower_frequency_json.test.csv").generic_string()});
    test_dataset.test_csv.add_function("time [s]", [&system_elasto] { return system_elasto.get_time(); });
    test_dataset.test_csv.add_function("top x [m]", [&tower] { return tower.nodes.back()->get_position().x(); });

    // decay
    double dt = 0.01;
    double duration = 10.0;
    tower.nodes.back()->set_force(Vector3d(-3000.0, 0.0, 0.0), false);
    system_elasto.do_statics(true, 10);
    tower.nodes.back()->set_force(Vector3d(0.0, 0.0, 0.0), false);
    while (system_elasto.get_time() <= duration) {
        test_dataset.test_csv.write_row();
        system_elasto.step(dt);
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
    ref_point1.damping_foreaft = 0.005;
    ref_point1.damping_sideside = 0.005;
    ref_point1.damping_axial = 0.005;
    ref_point1.damping_torsion = 0.005;
    ref_point1.damping_mass = 0.0;
    // top
    auto ref_point2 = seahowl::elasto::TowerReferencePointElasto();
    ref_point2.set_properties_cylinder(density, young_modulus, poisson_ratio, diameter, thickness, false);
    ref_point2.coordinates = seahowl::Vector3d(0.0, 0.0, 100.0);
    ref_point2.fraction = 1.0;
    ref_point2.damping_foreaft = 0.005;
    ref_point2.damping_sideside = 0.005;
    ref_point2.damping_axial = 0.005;
    ref_point2.damping_torsion = 0.005;
    ref_point2.damping_mass = 0.0;
    //
    tower.reference_points = {ref_point1, ref_point2};
    tower.discretization_fractions = {20};
    tower.build();
    tower.assemble(system_elasto);
    tower.nodes.front()->set_fixed(true);

    // Setup TestFwDataSet
    TestFrameworkDataset test_dataset({false, (ref_dir / "test_tower_cylinder_frequency.csv").generic_string(),
                                       (test_dir / "test_tower_cylinder_frequency.test.csv").generic_string()});
    test_dataset.test_csv.add_function("time [s]", [&system_elasto] { return system_elasto.get_time(); });
    test_dataset.test_csv.add_function("top x [m]", [&tower] { return tower.nodes.back()->get_position().x(); });

    // decay
    double dt = 0.01;
    double duration = 10.0;
    tower.nodes.back()->set_force(Vector3d(-3000.0, 0.0, 0.0), false);
    system_elasto.do_statics(true, 10);
    tower.nodes.back()->set_force(Vector3d(0.0, 0.0, 0.0), false);
    while (system_elasto.get_time() <= duration) {
        test_dataset.test_csv.write_row();
        system_elasto.step(dt);
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
    ref_point1.damping_foreaft = 0.005;
    ref_point1.damping_sideside = 0.005;
    ref_point1.damping_axial = 0.005;
    ref_point1.damping_torsion = 0.005;
    ref_point1.damping_mass = 0.0;
    // top
    auto ref_point2 = seahowl::elasto::TowerReferencePointElasto();
    ref_point2.set_properties_cylinder(density, young_modulus, poisson_ratio, 3.0, 0.015, false);
    ref_point2.coordinates = seahowl::Vector3d(0.0, 0.0, 100.0);
    ref_point2.fraction = 1.0;
    ref_point2.damping_foreaft = 0.005;
    ref_point2.damping_sideside = 0.005;
    ref_point2.damping_axial = 0.005;
    ref_point2.damping_torsion = 0.005;
    ref_point2.damping_mass = 0.0;
    //
    tower.reference_points = {ref_point1, ref_point2};
    tower.discretization_fractions = {20};
    tower.build();
    tower.assemble(system_elasto);
    tower.nodes.front()->set_fixed(true);

    // Setup TestFwDataSet
    TestFrameworkDataset test_dataset({false, (ref_dir / "test_tower_conical_frequency.csv").generic_string(),
                                       (test_dir / "test_tower_conical_frequency.test.csv").generic_string()});
    test_dataset.test_csv.add_function("time [s]", [&system_elasto] { return system_elasto.get_time(); });
    test_dataset.test_csv.add_function("top x [m]", [&tower] { return tower.nodes.back()->get_position().x(); });

    // decay
    double dt = 0.01;
    double duration = 10.0;
    tower.nodes.back()->set_force(Vector3d(-3000.0, 0.0, 0.0), false);
    system_elasto.do_statics(true, 10);
    tower.nodes.back()->set_force(Vector3d(0.0, 0.0, 0.0), false);
    while (system_elasto.get_time() <= duration) {
        test_dataset.test_csv.write_row();
        system_elasto.step(dt);
    }

    EvaluateTest(test_dataset);
}
