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

// The fixture for testing
class TestFloating : public FixtureComponents {
  protected:
    TestFloating() : FixtureComponents() {
        root_dir /= "test_floating";
        ref_dir /= "test_floating/ref";
        test_dir /= "test_floating/test";
    }

    // Create simulation with standard output settings disabled
    seahowl::core::Simulation create_simulation(double dt,
                                                double duration,
                                                const std::string& turbine_filepath,
                                                const std::string& env_filepath) {
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

        // add env
        system_core.env_model = seahowl::io::get_environmental_model_from_file(env_filepath);

        return simulation;
    }

    // Add common turbine metrics to test dataset
    void add_common_metrics(seahowl::core::Simulation& simulation, TestFrameworkDataset& dataset) {
        auto& system_core = *simulation.system_core;
        auto& turbine = *system_core.turbines[0];
        auto& floater = *std::dynamic_pointer_cast<seahowl::core::Floater>(turbine.foundation);
        dataset.test_csv.add_function("time [s]", [&system_core]() { return system_core.get_time(); });
        dataset.test_csv.add_function("rpm [-]", [&turbine]() { return turbine.rna.elasto.get_rpm(); });
        dataset.test_csv.add_function("cog [m]", [&floater]() { return floater.elasto.body_main->get_position(); });
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

        system_core.run_presimulation(5.0, 0.05, false, true);

        dataset.test_csv.write_row();  // write outputs at t=0.0
        // simulation loop
        while (system_core.get_time() < simulation.duration) {
            simulation.step();
            dataset.test_csv.write_row();
        }
    }
};

#ifdef HAVE_HYDRODYN
TEST_F(TestFloating, IEA15MW_hydrodyn) {
    // create simulation
    auto turbine_filepath = (DATADIR / "IEA15MW/floating/turbine_hydrodyn.json").generic_string();
    auto env_filepath = (root_dir / "assets/env_waves_200m_seastate_regular.json").generic_string();
    auto simulation = create_simulation(0.05, 50.0, turbine_filepath, env_filepath);

    // create test dataset
    TestFrameworkDataset test_dataset({false, (ref_dir / "test_IEA15MW_hydrodyn.csv").generic_string(),
                                       (test_dir / "test_IEA15MW_hydrodyn.test.csv").generic_string()});
    add_common_metrics(simulation, test_dataset);

    // run simulation
    run_simulation_loop(simulation, test_dataset);

    // evaluate test results
    EvaluateTest(test_dataset);
}
#endif  // HAVE_HYDRODYN

#ifdef HAVE_HYDROCHRONO
    #ifndef _MSC_VER  // segfault issue when using HydroChrono with MSVC here
TEST_F(TestFloating, IEA15MW_hydrochrono) {
    // create simulation
    auto turbine_filepath = (DATADIR / "IEA15MW/floating/turbine_hydrochrono.json").generic_string();
    auto env_filepath = (root_dir / "assets/env_waves_200m_hydrochrono_regular.json").generic_string();
    auto simulation = create_simulation(0.05, 50.0, turbine_filepath, env_filepath);

    // create test dataset
    TestFrameworkDataset test_dataset({false, (ref_dir / "test_IEA15MW_hydrochrono.csv").generic_string(),
                                       (test_dir / "test_IEA15MW_hydrochrono.test.csv").generic_string()});
    add_common_metrics(simulation, test_dataset);

    // run simulation
    run_simulation_loop(simulation, test_dataset);

    // evaluate test results
    EvaluateTest(test_dataset);
}
    #endif  // _MSC_VER
#endif      // HAVE_HYDROCHRONO
