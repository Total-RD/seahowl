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
#include <spdlog/spdlog.h>

// Standard library
#include <filesystem>

using std::filesystem::path;

using namespace seahowl;
using namespace seahowl::elasto;

// The fixture for testing
class TestTurbine : public FixtureComponents {
  protected:
    TestTurbine() : FixtureComponents() {
        root_dir /= "test_turbine";
        ref_dir /= "test_turbine/ref";
        test_dir /= "test_turbine/test";
    }

    // Create simulation with standard output settings disabled
    seahowl::core::Simulation create_simulation(double dt, double duration, const std::string& turbine_filepath) {
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

        // add wind model
        auto wind_model = std::make_shared<seahowl::env::ConstantWind>();
        wind_model->set_wind_velocity(Vector3d(8.0, 0.0, 0.0));
        wind_model->shear_coefficient = 0.12;
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
        dataset.test_csv.add_function("blade1 root moment [Nm]", [&turbine]() {
            return turbine.rna.elasto.rotor->blades[0]->get_blade_root_moment();
        });
    }

    // Run simulation loop writing to dataset each step
    void run_simulation_loop(seahowl::core::Simulation& simulation, TestFrameworkDataset& dataset) {
        auto& system_core = *simulation.system_core;

        // initialization
        simulation.initialize();

        // apply initial pitch
        for (auto& turbine : system_core.turbines) {
            turbine->rna.elasto.rotor->apply_collective_pitch_increment(seahowl::PI / 8.0);
        }

        // statics
        system_core.elasto.do_statics(true, 10);
        system_core.poststep(0.0, simulation.dt);

        // simulation loop
        while (system_core.get_time() < simulation.duration) {
            simulation.step();
            dataset.test_csv.write_row();
        }
    }
};

TEST_F(TestTurbine, IEA15MW_fixed_pitch_fea) {
    // create simulation
    auto turbine_filepath = (DATADIR / "IEA15MW/onshore/turbine.json").generic_string();
    auto simulation = create_simulation(0.1, 50.0, turbine_filepath);
    // apply fpm mode
    for (auto& blade : simulation.system_core->turbines[0]->elasto.rna->rotor->blades) {
        dynamic_cast<seahowl::elasto::BladeElastoFEA&>(*blade).fpm_mode = false;
    }
    // rebuild with chosen fpm mode
    simulation.system_core->turbines[0]->build();

    // create test dataset
    TestFrameworkDataset test_dataset({false, (ref_dir / "test_IEA15MW_fixed_pitch_fea.csv").generic_string(),
                                       (test_dir / "test_IEA15MW_fixed_pitch_fea.test.csv").generic_string()});
    add_common_metrics(simulation, test_dataset);

    // run simulation
    run_simulation_loop(simulation, test_dataset);

    // evaluate test results
    EvaluateTest(test_dataset);
}

TEST_F(TestTurbine, IEA15MW_fixed_pitch_fpm) {
    // create simulation
    auto turbine_filepath = (DATADIR / "IEA15MW/onshore/turbine.json").generic_string();
    auto simulation = create_simulation(0.1, 50.0, turbine_filepath);
    // apply fpm mode
    for (auto& blade : simulation.system_core->turbines[0]->elasto.rna->rotor->blades) {
        dynamic_cast<seahowl::elasto::BladeElastoFEA&>(*blade).fpm_mode = true;
    }
    // rebuild with chosen fpm mode
    simulation.system_core->turbines[0]->build();

    // create test dataset
    TestFrameworkDataset test_dataset({false, (ref_dir / "test_IEA15MW_fixed_pitch_fpm.csv").generic_string(),
                                       (test_dir / "test_IEA15MW_fixed_pitch_fpm.test.csv").generic_string()});
    add_common_metrics(simulation, test_dataset);

    // run simulation
    run_simulation_loop(simulation, test_dataset);

    // evaluate test results
    EvaluateTest(test_dataset);
}

TEST_F(TestTurbine, IEA15MW_fixed_pitch_rigid) {
    // create simulation
    auto turbine_filepath = (DATADIR / "IEA15MW/onshore/turbine_rigid.json").generic_string();
    auto simulation = create_simulation(0.1, 50.0, turbine_filepath);

    // create test dataset
    TestFrameworkDataset test_dataset({false, (ref_dir / "test_IEA15MW_fixed_pitch_rigid.csv").generic_string(),
                                       (test_dir / "test_IEA15MW_fixed_pitch_rigid.test.csv").generic_string()});
    add_common_metrics(simulation, test_dataset);

    // run simulation
    run_simulation_loop(simulation, test_dataset);

    // evaluate test results
    EvaluateTest(test_dataset);
}

TEST_F(TestTurbine, IEA15MW_target_rpm) {
    // create simulation
    auto turbine_filepath = (DATADIR / "IEA15MW/onshore/turbine_rigid.json").generic_string();
    auto simulation = create_simulation(0.05, 100.0, turbine_filepath);
    // add target RPM controller
    auto& system_core = *simulation.system_core;
    auto& turbine = *system_core.turbines[0];
    auto controller = std::make_shared<seahowl::servo::ControllerVariableTorque>();
    controller->set_target_rpm(2.0);
    turbine.controller = controller;

    // create test dataset
    TestFrameworkDataset test_dataset({false, (ref_dir / "test_IEA15MW_target_rpm.csv").generic_string(),
                                       (test_dir / "test_IEA15MW_target_rpm.test.csv").generic_string()});
    test_dataset.test_csv.add_function("time [s]", [&system_core]() { return system_core.get_time(); });
    test_dataset.test_csv.add_function("rpm [-]", [&turbine]() { return turbine.rna.elasto.get_rpm(); });

    // run simulation
    run_simulation_loop(simulation, test_dataset);

    // evaluate test results
    EvaluateTest(test_dataset);
}

TEST_F(TestTurbine, IEA15MW_actuator_disk) {
    // create simulation
    auto turbine_filepath = (DATADIR / "IEA15MW/onshore/turbine_disk.json").generic_string();
    auto simulation = create_simulation(0.1, 200.0, turbine_filepath);
    auto& system_core = *simulation.system_core;
    auto& turbine = *system_core.turbines[0];

    // add wind model
    auto env_model = std::make_shared<seahowl::env::EnvModel>();
    auto wind_model = std::make_shared<seahowl::env::ConstantWind>();
    wind_model->set_wind_velocity(Vector3d(11.0, 0.0, 0.0));
    wind_model->shear_coefficient = 0.12;
    env_model->add_model(wind_model);
    system_core.env_model = env_model;

    // add target controller
    auto controller = std::make_shared<seahowl::servo::ControllerVariableTorque>();
    controller->set_target_rpm(7.56);
    turbine.controller = controller;

    TestFrameworkDataset test_dataset({false, (ref_dir / "test_IEA15MW_actuator_disk.csv").generic_string(),
                                       (test_dir / "test_IEA15MW_actuator_disk.test.csv").generic_string()});
    test_dataset.test_csv.add_function("time [s]", [&system_core]() { return system_core.get_time(); });
    test_dataset.test_csv.add_function("rpm [-]", [&turbine]() { return turbine.rna.elasto.get_rpm(); });
    test_dataset.test_csv.add_function("power [W]", [&turbine]() { return turbine.get_generated_power(); });

    // initialization
    simulation.initialize();

    // statics
    system_core.elasto.do_statics(true, 10);
    system_core.poststep(0.0, simulation.dt);

    // Phase 1: wind 11 m/s
    int count = 0;
    while (system_core.get_time() < 100.0) {
        simulation.step();
        if (count % 5 == 0) {
            test_dataset.test_csv.write_row();
        }
        count += 1;
    }

    // Phase 2: wind 15 m/s with pitch adjustment
    wind_model->set_wind_velocity(Vector3d(15.0, 0.0, 0.0));
    turbine.rna.elasto.rotor->apply_collective_pitch_increment(11.0 * seahowl::PI / 180.0);
    count = 0;
    while (system_core.get_time() < 200) {
        simulation.step();
        if (count % 5 == 0) {
            test_dataset.test_csv.write_row();
        }
        count += 1;
    }

    // evaluate test results
    EvaluateTest(test_dataset);
}

TEST_F(TestTurbine, IEA15MW_multiturbines) {
    // create simulation
    auto turbine_filepath = (DATADIR / "IEA15MW/onshore/turbine_rigid.json").generic_string();
    auto simulation = create_simulation(0.1, 50.0, turbine_filepath);
    auto& system_core = *simulation.system_core;
    constexpr int nturbines = 2;
    for (int ii = 0; ii < nturbines; ii++) {
        seahowl::io::add_turbine_to_system_from_file(turbine_filepath, system_core);
        system_core.turbines[ii + 1]->controller = std::make_shared<seahowl::servo::Controller>();
    }

    TestFrameworkDataset test_dataset({false, (ref_dir / "test_IEA15MW_multiturbines.csv").generic_string(),
                                       (test_dir / "test_IEA15MW_multiturbines.test.csv").generic_string()});
    test_dataset.test_csv.add_function("time [s]", [&system_core]() { return system_core.get_time(); });
    for (size_t idx = 0; idx < system_core.turbines.size(); idx++) {
        test_dataset.test_csv.add_function("rpm turbine " + std::to_string(idx + 1) + " [-]", [&system_core, idx]() {
            return system_core.turbines[idx]->rna.elasto.get_rpm();
        });
    }

    // run simulation
    run_simulation_loop(simulation, test_dataset);

    // evaluate test results
    EvaluateTest(test_dataset);
}

TEST_F(TestTurbine, IEA34MW_fixed_pitch) {
    // create simulation
    auto turbine_filepath = (DATADIR / "IEA3.4MW/turbine/turbine.json").generic_string();
    auto simulation = create_simulation(0.1, 50.0, turbine_filepath);
    // apply fpm mode
    for (auto& blade : simulation.system_core->turbines[0]->elasto.rna->rotor->blades) {
        dynamic_cast<seahowl::elasto::BladeElastoFEA&>(*blade).fpm_mode = true;
    }
    // rebuild with chosen fpm mode
    simulation.system_core->turbines[0]->build();

    // create test dataset
    TestFrameworkDataset test_dataset({false, (ref_dir / "test_IEA34MW_fixed_pitch.csv").generic_string(),
                                       (test_dir / "test_IEA34MW_fixed_pitch.test.csv").generic_string()});
    add_common_metrics(simulation, test_dataset);

    // run simulation
    run_simulation_loop(simulation, test_dataset);

    // evaluate test results
    EvaluateTest(test_dataset);
}

TEST_F(TestTurbine, IEA10MW_fixed_pitch) {
    // create simulation
    auto turbine_filepath = (DATADIR / "IEA10MW/turbine/turbine.json").generic_string();
    auto simulation = create_simulation(0.1, 50.0, turbine_filepath);
    // apply fpm mode
    for (auto& blade : simulation.system_core->turbines[0]->elasto.rna->rotor->blades) {
        dynamic_cast<seahowl::elasto::BladeElastoFEA&>(*blade).fpm_mode = true;
    }
    // rebuild with chosen fpm mode
    simulation.system_core->turbines[0]->build();

    // create test dataset
    TestFrameworkDataset test_dataset({false, (ref_dir / "test_IEA10MW_fixed_pitch.csv").generic_string(),
                                       (test_dir / "test_IEA10MW_fixed_pitch.test.csv").generic_string()});
    add_common_metrics(simulation, test_dataset);

    // run simulation
    run_simulation_loop(simulation, test_dataset);

    // evaluate test results
    EvaluateTest(test_dataset);
}
