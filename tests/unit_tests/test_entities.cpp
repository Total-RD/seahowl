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
#include <seahowl/commons.h>
#include <seahowl/elasto.h>

// Third-party libraries
#include <gtest/gtest.h>

// Standard library
#include <filesystem>

using std::filesystem::path;

using namespace seahowl;
using namespace seahowl::elasto;

// The fixture for testing
class TestEntities : public FixtureComponents {
  protected:
    TestEntities() : FixtureComponents() {
        root_dir /= "test_entities";
        ref_dir /= "test_entities/ref";
        test_dir /= "test_entities/test";
    }
};

TEST_F(TestEntities, added_mass_damping) {
    double dt = 0.1;
    double duration = 30.0;

    // system
    auto system_elasto = SystemElastoChrono();
    system_elasto.set_gravitational_acceleration(Vector3d(0.0, 0.0, -9.81));

    // bodies
    std::vector<seahowl::elasto::BodyElastoChrono> bodies;
    for (int ii = 0; ii < 7; ii++) {
        // create body
        bodies.push_back(seahowl::elasto::BodyElastoChrono());
        auto& body = bodies.back();
        system_elasto.add(body);
        // mass and inertia
        body.set_mass(1.0);
        Eigen::Matrix<double, 3, 3> matrix_inertia = Eigen::Matrix<double, 3, 3>::Zero();
        matrix_inertia(0, 0) = 1.0;
        matrix_inertia(1, 1) = 1.0;
        matrix_inertia(2, 2) = 1.0;
        body.set_inertia_matrix(matrix_inertia);
    }

    double added_mass_value = 3.0;
    double damping_value = 0.1;
    Eigen::Matrix<double, 6, 6> matrix_am = Eigen::Matrix<double, 6, 6>::Zero();
    Eigen::Matrix<double, 6, 6> matrix_damping = Eigen::Matrix<double, 6, 6>::Zero();
    matrix_am(2, 2) = added_mass_value;
    matrix_am(5, 5) = added_mass_value;
    matrix_damping(2, 2) = damping_value;
    matrix_damping(5, 5) = damping_value;

    seahowl::Quaternion rot(seahowl::AngleAxisd(-seahowl::PI / 2.0, seahowl::Vector3d(0.0, 1.0, 0.0)));

    // body 1: added mass matrix
    bodies[1].set_added_mass_matrix(matrix_am);

    // body 2: added mass matrix and rotation
    bodies[2].set_added_mass_matrix(matrix_am);
    bodies[2].set_rotation(rot);

    // body 3: damping matrix
    bodies[3].set_damping_matrix(matrix_damping);

    // body 4: damping matrix and rotation
    bodies[4].set_damping_matrix(matrix_damping);
    bodies[4].set_rotation(rot);

    // body 5: damping and added mass matrices
    bodies[5].set_damping_matrix(matrix_damping);
    bodies[5].set_added_mass_matrix(matrix_am);

    // body 6: damping and added mass matrices and rotation
    bodies[6].set_damping_matrix(matrix_damping);
    bodies[6].set_added_mass_matrix(matrix_am);
    bodies[6].set_rotation(rot);

    for (auto& body : bodies) {
        body.set_torque(seahowl::Vector3d(0.0, 0.0, 10.0), false);  // add torque in global Z
    }

    double current_time = 0.0;

    // Setup TestFwDataSet
    TestFrameworkDataset test_dataset({false, (ref_dir / "test_entities_added_mass_damping.csv").generic_string(),
                                       (test_dir / "test_entities_added_mass_damping.test.csv").generic_string()});

    test_dataset.test_csv.add_function("time [s]", [&current_time] { return current_time; });
    int idx_body = 0;
    for (auto& body : bodies) {
        test_dataset.test_csv.add_function("z pos body" + std::to_string(idx_body) + " [m]",
                                           [&body] { return body.get_position(); });
        test_dataset.test_csv.add_function("velocity rotation body" + std::to_string(idx_body) + " [rad/s]",
                                           [&body] { return body.get_rotational_velocity(true); });
        idx_body += 1;
    }

    // loop
    while (current_time <= duration) {
        test_dataset.test_csv.write_row();
        system_elasto.step(dt);
        current_time += dt;
    }

    EvaluateTest(test_dataset);
}
