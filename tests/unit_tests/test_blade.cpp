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
class TestBlade : public FixtureComponents {
  protected:
    TestBlade() : FixtureComponents() {
        root_dir /= "test_blade";
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
    blade.reference_points = seahowl::io::get_blade_elasto_reference_points_from_file(
        (DATADIR / "IEA15MW/base/blade.json").generic_string());
    // make 50 elements
    blade.discretization_fractions.clear();
    for (int ii = 0; ii < 51; ii++) {
        blade.discretization_fractions.push_back(0.02 * ii);
    }
    blade.build();
    blade.assemble(system_elasto);
    blade.nodes.front()->set_fixed(true);

    // Setup TestFwDataSet
    TestFrameworkDataset test_dataset({false, (ref_dir / "test_blade_geometry.csv").generic_string(),
                                       (test_dir / "test_blade_geometry.test.csv").generic_string()});
    auto& node = blade.nodes.front();
    test_dataset.test_csv.add_function("position [m]", [&node] { return node->get_position(); });
    // check geometry
    for (int ii = 0; ii < blade.nodes.size(); ii++) {
        node = blade.nodes[ii];
        test_dataset.test_csv.write_row();
    }

    // statics
    system_elasto.do_statics(true, 0);

    // check mass
    TestFrameworkDataset test_dataset_mass({false, (ref_dir / "test_blade_mass.csv").generic_string(),
                                            (test_dir / "test_blade_mass.test.csv").generic_string()});
    test_dataset_mass.test_csv.add_function("mass [kg]", [&blade] { return blade.get_mass(); });
    test_dataset_mass.test_csv.write_row();
    EvaluateTest(test_dataset_mass);
}

TEST_F(TestBlade, edgewise) {
    // system
    auto system_elasto = SystemElastoChrono();
    system_elasto.set_gravitational_acceleration(Vector3d(0.0, 0.0, -9.81));

    // blade
    auto blade = seahowl::elasto::BladeElastoFEA();
    blade.fpm_mode = true;
    seahowl::io::populate_blade_elasto_from_file((DATADIR / "IEA15MW/base/blade.json").generic_string(), blade);
    // make 50 elements
    blade.discretization_fractions.clear();
    for (int ii = 0; ii < 51; ii++) {
        blade.discretization_fractions.push_back(0.02 * ii);
    }
    blade.build();
    blade.assemble(system_elasto);
    blade.nodes.front()->set_fixed(true);

    // get equilibrium position
    system_elasto.do_statics(true, 10);
    auto pos_equilibrium = blade.nodes.back()->get_position();

    // Setup TestFwDataSet
    TestFrameworkDataset test_dataset({false, (ref_dir / "test_blade_edgewise.csv").generic_string(),
                                       (test_dir / "test_blade_edgewise.test.csv").generic_string()});
    test_dataset.test_csv.add_function("time [s]", [&system_elasto] { return system_elasto.get_time(); });
    test_dataset.test_csv.add_function("tip y [m]", [&blade, &pos_equilibrium] {
        return blade.nodes.back()->get_position().y() - pos_equilibrium.y();
    });

    // decay
    double dt = 0.1;
    double duration = 100.0;
    blade.nodes.back()->set_force(Vector3d(0.0, 300.0, 0.0), false);
    system_elasto.do_statics(true, 10);
    blade.nodes.back()->set_force(Vector3d(0.0, 0.0, 0.0), false);
    while (system_elasto.get_time() <= duration) {
        test_dataset.test_csv.write_row();
        system_elasto.step(dt);
    }

    EvaluateTest(test_dataset);

    // Setup TestFwDataSet
    TestFrameworkDataset test_dataset_deflection(
        {false, (ref_dir / "test_blade_edgewise_deflection.csv").generic_string(),
         (test_dir / "test_blade_edgewise_deflection.test.csv").generic_string()});
    test_dataset_deflection.test_csv.add_function("deflection [m]",
                                                  [&blade] { return blade.nodes.back()->get_position().z(); });

    // rotate blade (flat along y axis)
    blade.rotate(PI / 2.0, Vector3d(1.0, 0.0, 0.0));
    // test deflection
    system_elasto.do_statics(true, 10);
    test_dataset_deflection.test_csv.write_row();

    // flip blade
    blade.rotate(PI, Vector3d(0.0, 1.0, 0.0));
    // taste deflection
    system_elasto.do_statics(true, 10);
    test_dataset_deflection.test_csv.write_row();

    // check that deflections match reference
    EvaluateTest(test_dataset_deflection);
}

TEST_F(TestBlade, flapwise) {
    // system
    auto system_elasto = SystemElastoChrono();
    system_elasto.set_gravitational_acceleration(Vector3d(0.0, 0.0, -9.81));

    // blade
    auto blade = seahowl::elasto::BladeElastoFEA();
    blade.fpm_mode = true;
    seahowl::io::populate_blade_elasto_from_file((DATADIR / "IEA15MW/base/blade.json").generic_string(), blade);
    // make 50 elements
    blade.discretization_fractions.clear();
    for (int ii = 0; ii < 51; ii++) {
        blade.discretization_fractions.push_back(0.02 * ii);
    }
    blade.build();
    blade.assemble(system_elasto);
    blade.nodes.front()->set_fixed(true);

    // get equilibrium position
    system_elasto.do_statics(true, 10);
    auto pos_equilibrium = blade.nodes.back()->get_position();

    // Setup TestFwDataSet
    TestFrameworkDataset test_dataset({false, (ref_dir / "test_blade_flapwise.csv").generic_string(),
                                       (test_dir / "test_blade_flapwise.test.csv").generic_string()});
    test_dataset.test_csv.add_function("time [s]", [&system_elasto] { return system_elasto.get_time(); });
    test_dataset.test_csv.add_function("tip x [m]", [&blade, &pos_equilibrium] {
        return blade.nodes.back()->get_position().x() - pos_equilibrium.x();
    });

    // decay
    double dt = 0.1;
    double duration = 100.0;
    blade.nodes.back()->set_force(Vector3d(-300.0, 0.0, 0.0), false);
    system_elasto.do_statics(true, 10);
    blade.nodes.back()->set_force(Vector3d(0.0, 0.0, 0.0), false);
    while (system_elasto.get_time() <= duration) {
        test_dataset.test_csv.write_row();
        system_elasto.step(dt);
    }

    EvaluateTest(test_dataset);

    // Setup TestFwDataSet
    TestFrameworkDataset test_dataset_deflection(
        {false, (ref_dir / "test_blade_flapwise_deflection.csv").generic_string(),
         (test_dir / "test_blade_flapwise_deflection.test.csv").generic_string()});
    test_dataset_deflection.test_csv.add_function("deflection [m]",
                                                  [&blade] { return blade.nodes.back()->get_position().z(); });

    // rotate blade (flat along x axis)
    blade.rotate(PI / 2.0, Vector3d(0.0, 1.0, 0.0));
    // test deflection
    system_elasto.do_statics(true, 10);
    test_dataset_deflection.test_csv.write_row();

    // flip blade
    blade.rotate(PI, Vector3d(1.0, 0.0, 0.0));
    // test deflection
    system_elasto.do_statics(true, 10);
    test_dataset_deflection.test_csv.write_row();

    EvaluateTest(test_dataset_deflection);
}
