// Local test headers
#include "fixture_components.h"

// SEAHOWL headers
#include <seahowl/core.h>
#include <seahowl/elasto.h>
#include <seahowl/env.h>
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
class TestController : public FixtureComponents {
  protected:
    TestController() : FixtureComponents() {
        root_dir /= "test_controller";
        ref_dir /= "test_controller/ref";
        test_dir /= "test_controller/test";
    }

    // Create simulation with standard output settings disabled
    seahowl::core::Simulation create_simulation(double dt,
                                                double duration,
                                                const std::string& turbine_filepath,
                                                const Vector3d& velocity,
                                                bool has_actuator_dynamics) {
        seahowl::core::Simulation simulation;
        simulation.dt = dt;
        simulation.duration = duration;
        simulation.outputs->dt_output = 9999.9;
        simulation.outputs->has_csv = false;
        simulation.outputs->has_gui = false;
        simulation.outputs->has_vtk = false;

        auto& system_core = *simulation.system_core;
        auto& system_elasto = system_core.elasto;

        seahowl::io::add_turbine_to_system_from_file(turbine_filepath, system_core);

        auto& turbine = *system_core.turbines[0];
        for (auto& blade : turbine.rna.rotor.blades) {
            blade->elasto.actuator_pitch->set_fixed_actuator(!has_actuator_dynamics);
            blade->elasto.apply_pitch_increment(0.2);
        }

        auto wind_model = std::make_shared<seahowl::env::ConstantWind>();
        wind_model->shear_coefficient = 0.12;
        wind_model->reference_height = turbine.elasto.rna->rotor->body_hub->get_position().z();
        wind_model->set_wind_velocity(velocity);
        system_core.env_model->add_model(wind_model);

        return simulation;
    }

    // Run simulation loop writing to dataset each step
    void run_simulation_loop(seahowl::core::Simulation& simulation, TestFrameworkDataset& dataset) {
        auto& system_core = *simulation.system_core;

        // initialization
        simulation.initialize();
        system_core.elasto.do_statics(true, 10);
        system_core.poststep(0.0, simulation.dt);  // update aero positions before starting sim
        dataset.test_csv.write_row();              // write outputs at t=0.0

        // simulation loop
        while (system_core.get_time() < simulation.duration) {
            simulation.step();
            dataset.test_csv.write_row();
        }
    }
};

TEST_F(TestController, IEA15MW_CPC) {
    auto simulation = create_simulation(0.05, 200.0, (DATADIR / "IEA15MW/onshore/turbine.json").generic_string(),
                                        seahowl::Vector3d(12.0, 0.0, 0.0), true);
    auto& system_core = *simulation.system_core;
    auto& turbine = *system_core.turbines[0];

    TestFrameworkDataset test_dataset({false, (ref_dir / "test_IEA15MW_CPC.csv").generic_string(),
                                       (test_dir / "test_IEA15MW_CPC.test.csv").generic_string()});
    test_dataset.test_csv.add_function("time [s]", [&system_core]() { return system_core.get_time(); });
    test_dataset.test_csv.add_function("power [W]", [&turbine]() { return turbine.get_generated_power(); });
    test_dataset.test_csv.add_function("rpm [-]", [&turbine]() { return turbine.rna.elasto.get_rpm(); });
    test_dataset.test_csv.add_function("blade1 pitch [rad]",
                                       [&turbine]() { return turbine.rna.rotor.blades[0]->elasto.get_pitch(); });
    test_dataset.test_csv.add_function("blade1 root moment [Nm]", [&turbine]() {
        return turbine.rna.rotor.blades[0]->elasto.get_blade_root_moment();
    });

    run_simulation_loop(simulation, test_dataset);
    EvaluateTest(test_dataset);
};

TEST_F(TestController, IEA10MW_CPC) {
    auto simulation = create_simulation(0.05, 200.0, (DATADIR / "IEA10MW/turbine/turbine.json").generic_string(),
                                        seahowl::Vector3d(12.0, 0.0, 0.0), true);
    auto& system_core = *simulation.system_core;
    auto& turbine = *system_core.turbines[0];

    TestFrameworkDataset test_dataset({false, (ref_dir / "test_IEA10MW_CPC.csv").generic_string(),
                                       (test_dir / "test_IEA10MW_CPC.test.csv").generic_string()});
    test_dataset.test_csv.add_function("time [s]", [&system_core]() { return system_core.get_time(); });
    test_dataset.test_csv.add_function("power [W]", [&turbine]() { return turbine.get_generated_power(); });
    test_dataset.test_csv.add_function("rpm [-]", [&turbine]() { return turbine.rna.elasto.get_rpm(); });
    test_dataset.test_csv.add_function("blade1 pitch [rad]",
                                       [&turbine]() { return turbine.rna.rotor.blades[0]->elasto.get_pitch(); });
    test_dataset.test_csv.add_function("blade1 root moment [Nm]", [&turbine]() {
        return turbine.rna.rotor.blades[0]->elasto.get_blade_root_moment();
    });

    run_simulation_loop(simulation, test_dataset);
    EvaluateTest(test_dataset);
};

TEST_F(TestController, IEA34MW_CPC) {
    auto simulation = create_simulation(0.05, 200.0, (DATADIR / "IEA3.4MW/turbine/turbine.json").generic_string(),
                                        seahowl::Vector3d(12.0, 0.0, 0.0), true);
    auto& system_core = *simulation.system_core;
    auto& turbine = *system_core.turbines[0];

    TestFrameworkDataset test_dataset({false, (ref_dir / "test_IEA34MW_CPC.csv").generic_string(),
                                       (test_dir / "test_IEA34MW_CPC.test.csv").generic_string()});
    test_dataset.test_csv.add_function("time [s]", [&system_core]() { return system_core.get_time(); });
    test_dataset.test_csv.add_function("power [W]", [&turbine]() { return turbine.get_generated_power(); });
    test_dataset.test_csv.add_function("rpm [-]", [&turbine]() { return turbine.rna.elasto.get_rpm(); });
    test_dataset.test_csv.add_function("blade1 pitch [rad]",
                                       [&turbine]() { return turbine.rna.rotor.blades[0]->elasto.get_pitch(); });
    test_dataset.test_csv.add_function("blade1 root moment [Nm]", [&turbine]() {
        return turbine.rna.rotor.blades[0]->elasto.get_blade_root_moment();
    });

    run_simulation_loop(simulation, test_dataset);
    EvaluateTest(test_dataset);
};

TEST_F(TestController, IEA15MW_CPC_snap) {
    auto simulation = create_simulation(0.05, 200.0, (DATADIR / "IEA15MW/onshore/turbine.json").generic_string(),
                                        seahowl::Vector3d(12.0, 0.0, 0.0), false);
    auto& system_core = *simulation.system_core;
    auto& turbine = *system_core.turbines[0];

    TestFrameworkDataset test_dataset({false, (ref_dir / "test_IEA15MW_CPC_snap.csv").generic_string(),
                                       (test_dir / "test_IEA15MW_CPC_snap.test.csv").generic_string()});
    test_dataset.test_csv.add_function("time [s]", [&system_core]() { return system_core.get_time(); });
    test_dataset.test_csv.add_function("power [W]", [&turbine]() { return turbine.get_generated_power(); });
    test_dataset.test_csv.add_function("rpm [-]", [&turbine]() { return turbine.rna.elasto.get_rpm(); });
    test_dataset.test_csv.add_function("blade1 pitch [rad]",
                                       [&turbine]() { return turbine.rna.rotor.blades[0]->elasto.get_pitch(); });
    test_dataset.test_csv.add_function("blade1 root moment [Nm]", [&turbine]() {
        return turbine.rna.rotor.blades[0]->elasto.get_blade_root_moment();
    });

    run_simulation_loop(simulation, test_dataset);
    EvaluateTest(test_dataset);
};

TEST_F(TestController, IEA15MW_IPC) {
    auto simulation = create_simulation(0.05, 200.0, (DATADIR / "IEA15MW/onshore/turbine_ipc.json").generic_string(),
                                        seahowl::Vector3d(12.0, 0.0, 0.0), false);
    auto& system_core = *simulation.system_core;
    auto& turbine = *system_core.turbines[0];

    TestFrameworkDataset test_dataset({false, (ref_dir / "test_IEA15MW_IPC.csv").generic_string(),
                                       (test_dir / "test_IEA15MW_IPC.test.csv").generic_string()});
    test_dataset.test_csv.add_function("time [s]", [&system_core]() { return system_core.get_time(); });
    test_dataset.test_csv.add_function("power [W]", [&turbine]() { return turbine.get_generated_power(); });
    test_dataset.test_csv.add_function("rpm [-]", [&turbine]() { return turbine.rna.elasto.get_rpm(); });
    test_dataset.test_csv.add_function("blade1 pitch [rad]",
                                       [&turbine]() { return turbine.rna.rotor.blades[0]->elasto.get_pitch(); });
    test_dataset.test_csv.add_function("blade1 root moment [Nm]", [&turbine]() {
        return turbine.rna.rotor.blades[0]->elasto.get_blade_root_moment();
    });

    run_simulation_loop(simulation, test_dataset);
    EvaluateTest(test_dataset);
};

TEST_F(TestController, IEA15MW_yaw_control) {
    auto simulation = create_simulation(0.05, 50.0, (DATADIR / "IEA15MW/onshore/turbine_rigid.json").generic_string(),
                                        seahowl::Vector3d(12.0, 1.0, 0.0), true);
    auto& system_core = *simulation.system_core;
    auto& turbine = *system_core.turbines[0];

    auto controller = std::make_shared<seahowl::servo::ControllerDISCON>(
        (DATADIR / "IEA15MW/base/controller/DISCON_yaw.IN").generic_string(),
        (DATADIR / "IEA15MW/base/controller/libdiscon.so").generic_string());
    turbine.controller = controller;

    turbine.rna.elasto.actuator_yaw->set_fixed_actuator(false);
    turbine.rna.elasto.apply_yaw_increment(-0.1);

    TestFrameworkDataset test_dataset({false, (ref_dir / "test_IEA15MW_yaw_control.csv").generic_string(),
                                       (test_dir / "test_IEA15MW_yaw_control.test.csv").generic_string()});
    test_dataset.test_csv.add_function("time [s]", [&system_core]() { return system_core.get_time(); });
    test_dataset.test_csv.add_function("power [W]", [&turbine]() { return turbine.get_generated_power(); });
    test_dataset.test_csv.add_function("rpm [-]", [&turbine]() { return turbine.rna.elasto.get_rpm(); });
    test_dataset.test_csv.add_function("yaw [rad]", [&turbine]() { return turbine.rna.elasto.get_yaw(); });
    test_dataset.test_csv.add_function("towerbase moment [N]",
                                       [&turbine]() { return turbine.tower.elasto.get_tower_base_moment(); });

    run_simulation_loop(simulation, test_dataset);
    EvaluateTest(test_dataset);
};

TEST_F(TestController, IEA15MW_yaw_control_snap) {
    auto simulation = create_simulation(0.05, 50.0, (DATADIR / "IEA15MW/onshore/turbine_rigid.json").generic_string(),
                                        seahowl::Vector3d(12.0, 1.0, 0.0), true);
    auto& system_core = *simulation.system_core;
    auto& turbine = *system_core.turbines[0];

    auto controller = std::make_shared<seahowl::servo::ControllerDISCON>(
        (DATADIR / "IEA15MW/base/controller/DISCON_yaw.IN").generic_string(),
        (DATADIR / "IEA15MW/base/controller/libdiscon.so").generic_string());
    turbine.controller = controller;

    turbine.rna.elasto.actuator_yaw->set_fixed_actuator(true);
    turbine.rna.elasto.apply_yaw_increment(-0.1);

    TestFrameworkDataset test_dataset({false, (ref_dir / "test_IEA15MW_yaw_control_snap.csv").generic_string(),
                                       (test_dir / "test_IEA15MW_yaw_control_snap.test.csv").generic_string()});
    test_dataset.test_csv.add_function("time [s]", [&system_core]() { return system_core.get_time(); });
    test_dataset.test_csv.add_function("power [W]", [&turbine]() { return turbine.get_generated_power(); });
    test_dataset.test_csv.add_function("rpm [-]", [&turbine]() { return turbine.rna.elasto.get_rpm(); });
    test_dataset.test_csv.add_function("yaw [rad]", [&turbine]() { return turbine.rna.elasto.get_yaw(); });
    test_dataset.test_csv.add_function("towerbase moment [N]",
                                       [&turbine]() { return turbine.tower.elasto.get_tower_base_moment(); });

    run_simulation_loop(simulation, test_dataset);
    EvaluateTest(test_dataset);
};

TEST_F(TestController, IEA15MW_libdiscon_50turbines) {
    auto simulation = create_simulation(0.05, 0.1, (DATADIR / "IEA15MW/onshore/turbine_rigid.json").generic_string(),
                                        seahowl::Vector3d(12.0, 0.0, 0.0), true);
    auto& system_core = *simulation.system_core;

    for (int ii = 0; ii < 49; ii++) {
        seahowl::io::add_turbine_to_system_from_file((DATADIR / "IEA15MW/onshore/turbine_rigid.json").generic_string(),
                                                     system_core);
        system_core.turbines.back()->elasto.translate(seahowl::Vector3d(200.0 * (ii + 1), 200.0 * (ii + 1), 0.0));
    }

    simulation.initialize();

    while (system_core.get_time() < simulation.duration) {
        simulation.step();
    }
    // this test will throw an error if discon libraries cannot be loaded.
}

TEST_F(TestController, IEA15MW_actuator_disk_control) {
    auto simulation = create_simulation(0.05, 350.0, (DATADIR / "IEA15MW/onshore/turbine_disk.json").generic_string(),
                                        seahowl::Vector3d(12.0, 0.0, 0.0), true);
    auto& system_core = *simulation.system_core;
    auto& turbine = *system_core.turbines[0];

    auto wind_model = std::make_shared<seahowl::env::WindRamp>();
    wind_model->set_wind_ramp(Vector3d(5.0, 0.0, 0.0), 0.0, Vector3d(15.0, 0.0, 0.0), 250.0);
    wind_model->shear_coefficient = 0.12;
    system_core.env_model->add_model(wind_model);

    TestFrameworkDataset test_dataset({false, (ref_dir / "test_IEA15MW_actuator_disk_control.csv").generic_string(),
                                       (test_dir / "test_IEA15MW_actuator_disk_control.test.csv").generic_string()});
    test_dataset.test_csv.add_function("time [s]", [&system_core]() { return system_core.get_time(); });
    test_dataset.test_csv.add_function("power [W]", [&turbine]() { return turbine.get_generated_power(); });
    test_dataset.test_csv.add_function("rpm [-]", [&turbine]() { return turbine.rna.elasto.get_rpm(); });
    test_dataset.test_csv.add_function("pitch [rad]",
                                       [&turbine]() { return turbine.rna.elasto.rotor->pitch_collective; });

    run_simulation_loop(simulation, test_dataset);
    EvaluateTest(test_dataset);
}
