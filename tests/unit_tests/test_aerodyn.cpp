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
#include <memory>

using std::filesystem::path;

using namespace seahowl;
using namespace seahowl::elasto;

// The fixture for testing
class TestAeroDyn : public FixtureComponents {
  protected:
    TestAeroDyn() : FixtureComponents() {
        root_dir /= "test_aerodyn";
        ref_dir /= "test_aerodyn/ref";
        test_dir /= "test_aerodyn/test";
    }

    // Create simulation with standard output settings disabled
    seahowl::core::Simulation create_simulation(double dt,
                                                double duration,
                                                const std::string& turbine_filepath,
                                                const std::string& ifw_filepath) {
        // create simulation object
        seahowl::core::Simulation simulation;
        simulation.dt = dt;
        simulation.duration = duration;
        simulation.outputs->dt_output = 9999.9;
        simulation.outputs->has_csv = false;
        simulation.outputs->has_gui = false;
        simulation.outputs->has_vtk = false;

        auto& system_core = *simulation.system_core;
        // add turbine
        seahowl::io::add_turbine_to_system_from_file(turbine_filepath, system_core);
        auto& turbine = *system_core.turbines[0];
        // remove controller
        turbine.controller = std::make_shared<seahowl::servo::Controller>();
        // add inflowwind model
        auto wind_model = std::make_shared<seahowl::env::InflowWindAdapter>(ifw_filepath);
        system_core.env_model->add_model(wind_model);

        return simulation;
    }

    // Add common turbine metrics to test dataset
    void add_common_metrics(seahowl::core::Simulation& simulation, TestFrameworkDataset& dataset) {
        auto& system_core = *simulation.system_core;
        auto& turbine = *system_core.turbines[0];
        dataset.test_csv.add_function("time [s]", [&system_core]() { return system_core.get_time(); });
        dataset.test_csv.add_function("rpm [-]", [&turbine]() { return turbine.rna.elasto.get_rpm(); });
        dataset.test_csv.add_function("axial torque [Nm]",
                                      [&turbine]() { return turbine.rna.elasto.get_axial_torque(); });
    }

    // Run simulation loop writing to dataset each step
    void run_simulation_loop(seahowl::core::Simulation& simulation, TestFrameworkDataset& dataset) {
        auto& system_core = *simulation.system_core;
        auto& turbine = *system_core.turbines[0];

        // initialization
        simulation.initialize();

        // apply initial pitch
        for (auto& blade : turbine.rna.rotor.blades) {
            blade->elasto.apply_pitch_increment(seahowl::PI / 8.0);
        }

        // statics
        system_core.elasto.do_statics(true, 10);
        system_core.poststep(0.0, simulation.dt);

        dataset.test_csv.write_row();  // write outputs at t=0.0
        // simulation loop
        while (system_core.get_time() < simulation.duration) {
            simulation.step();
            dataset.test_csv.write_row();
        }
    }
};

TEST_F(TestAeroDyn, IEA15MW_fixed_pitch) {
    // create simulation
    auto turbine_filepath = (DATADIR / "IEA15MW/onshore/turbine_aerodyn.json").generic_string();
    auto inflowwind_filepath = (DATADIR / "IEA15MW/env/InflowWind.dat").generic_string();
    auto simulation = create_simulation(0.1, 50.0, turbine_filepath, inflowwind_filepath);

    // create test dataset
    TestFrameworkDataset test_dataset({false, (ref_dir / "test_IEA15MW_fixed_pitch.csv").generic_string(),
                                       (test_dir / "test_IEA15MW_fixed_pitch.test.csv").generic_string()});
    add_common_metrics(simulation, test_dataset);

    // run simulation
    run_simulation_loop(simulation, test_dataset);

    // evaluate test results
    EvaluateTest(test_dataset);
}

TEST_F(TestAeroDyn, IEA34MW_fixed_pitch) {
    // create simulation
    auto turbine_filepath = (DATADIR / "IEA3.4MW/turbine/turbine_aerodyn.json").generic_string();
    auto inflowwind_filepath = (DATADIR / "IEA3.4MW/env/InflowWind.dat").generic_string();
    auto simulation = create_simulation(0.1, 50.0, turbine_filepath, inflowwind_filepath);

    // create test dataset
    TestFrameworkDataset test_dataset({false, (ref_dir / "test_IEA34MW_fixed_pitch.csv").generic_string(),
                                       (test_dir / "test_IEA34MW_fixed_pitch.test.csv").generic_string()});
    add_common_metrics(simulation, test_dataset);

    // run simulation
    run_simulation_loop(simulation, test_dataset);

    // evaluate test results
    EvaluateTest(test_dataset);
}

TEST_F(TestAeroDyn, IEA10MW_fixed_pitch) {
    // create simulation
    auto turbine_filepath = (DATADIR / "IEA10MW/turbine/turbine_aerodyn.json").generic_string();
    auto inflowwind_filepath = (DATADIR / "IEA10MW/env/InflowWind.dat").generic_string();
    auto simulation = create_simulation(0.1, 50.0, turbine_filepath, inflowwind_filepath);

    // create test dataset
    TestFrameworkDataset test_dataset({false, (ref_dir / "test_IEA10MW_fixed_pitch.csv").generic_string(),
                                       (test_dir / "test_IEA10MW_fixed_pitch.test.csv").generic_string()});
    add_common_metrics(simulation, test_dataset);

    // run simulation
    run_simulation_loop(simulation, test_dataset);

    // evaluate test results
    EvaluateTest(test_dataset);
}
