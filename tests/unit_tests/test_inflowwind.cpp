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
#include <seahowl/core.h>
#include <seahowl/elasto.h>
#include <seahowl/env.h>
#include <seahowl/fluid.h>
#include <seahowl/io.h>
#include <seahowl/servo.h>

// Third-party libraries
#include <gtest/gtest.h>

// Standard library
#include <filesystem>

using std::filesystem::path;

using namespace seahowl;
using namespace seahowl::elasto;

// The fixture for testing
class TestInflowWind : public FixtureComponents {
  protected:
    TestInflowWind() : FixtureComponents() {
        root_dir /= "test_inflowwind";
        ref_dir /= "test_inflowwind/ref";
        test_dir /= "test_inflowwind/test";
    }
};

TEST_F(TestInflowWind, rpm_initial_pitch) {
    // general options
    bool visualization_on = true;
    bool statics_prestep = true;
    // solver
    auto verbose = false;
    // timestepping
    double dt = 0.1;

    auto path = (DATADIR / "IEA15MW/env/InflowWind.dat").generic_string();
    // wind
    auto wind_model = std::make_shared<seahowl::env::InflowWindAdapter>(path);
    // env_model
    auto env_model = seahowl::env::EnvModel();
    env_model.add_model(wind_model);

    // turbine
    double initial_pitch = seahowl::PI / 8.0;

    // system
    auto system_elasto = SystemElastoChrono();
    system_elasto.set_gravitational_acceleration(Vector3d(0.0, 0.0, -9.81));

    // turbine
    auto turbine = seahowl::io::get_turbine_from_file((DATADIR / "IEA15MW/onshore/turbine.json").generic_string());
    // remove controller
    turbine.controller = std::make_shared<seahowl::servo::Controller>();

    // remove controller
    turbine.controller = std::make_shared<seahowl::servo::Controller>();
    turbine.build();
    double time = 0.0;
    turbine.elasto.assemble(system_elasto);
    turbine.initialize(time, dt);

    // apply pitch before statics
    turbine.rna.elasto.rotor->apply_collective_pitch_increment(initial_pitch);

    // statics
    if (statics_prestep) {
        turbine.rna.elasto.link_shaft_hub->set_constraints(true, true, true, true, true, true);
        system_elasto.do_statics(true, 10);
        turbine.rna.elasto.link_shaft_hub->set_constraints(true, true, true, false, true, true);
    }
    turbine.poststep(0.0, dt);  // to update positions aero after statics step

    // Setup TestFwDataSet
    TestFrameworkDataset test_dataset({false, (ref_dir / "test_inflowwind_rpm_initial_pitch.csv").generic_string(),
                                       (test_dir / "test_inflowwind_rpm_initial_pitch.test.csv").generic_string()});
    test_dataset.test_csv.add_function("time [s]", [&system_elasto]() { return system_elasto.get_time(); });
    test_dataset.test_csv.add_function("rpm [-]", [&turbine]() { return turbine.rna.elasto.get_rpm(); });

    while (time < 50.0) {
        // prestep
        // compute forces
        turbine.apply_control(time, dt);
        turbine.fluid.compute_env_loads(env_model, time);
        // prestep (accumulates loads from aero to elasto)
        turbine.prestep(time, dt);

        system_elasto.step(dt);
        time += dt;

        // poststep
        turbine.poststep(time, dt);
        test_dataset.test_csv.write_row();
    }

    EvaluateTest(test_dataset);
}
