#include "fixture_components.h"

#include <gtest/gtest.h>
#include <spdlog/spdlog.h>
#include <seahowl/elasto/chrono_adapters.h>
#include <seahowl/elasto/blade_elasto.h>
#include <seahowl/env/wind_models.h>
#include <seahowl/env/env_model.h>
#include <seahowl/commons/numerics.h>
#include <seahowl/aero/turbine_aero.h>
#include <seahowl/core/turbine.h>
#include <seahowl/core/system.h>
#include <seahowl/servo/controller.h>
#include <seahowl/aero/system_aero.h>
#include <seahowl/io/read_input.h>

#include <filesystem>  // C++17
using std::filesystem::path;

using namespace seahowl;
using namespace seahowl::elasto;

// The fixture for testing
class TestTurbine : public FixtureComponents {
  protected:
    TestTurbine() : FixtureComponents() {
        ref_dir /= "test_turbine/ref";
        test_dir /= "test_turbine/test";
    }
};

TEST_F(TestTurbine, rpm_initial_pitch) {
    // general options
    bool visualization_on = true;
    bool statics_prestep = true;
    // solver
    auto verbose = false;
    // timestepping
    double dt = 0.1;
    // wind
    auto wind_model = std::make_shared<seahowl::env::ConstantWind>();
    wind_model->set_wind_velocity(Vector3d(8.0, 0.0, 0.0));
    wind_model->shear_coefficient = 0.12;
    // env_model
    auto env_model = seahowl::env::EnvModel();
    env_model.addModel(wind_model);

    // turbine
    double initial_pitch = seahowl::PI / 8.0;

    // system
    auto system_elasto = SystemElastoChrono();
    system_elasto.set_gravitational_acceleration(Vector3d(0.0, 0.0, -9.81));
    auto system_chrono = system_elasto.chobj;

    // turbine
    auto turbine_elasto = seahowl::elasto::TurbineElasto();
    auto turbine_aero = seahowl::aero::TurbineAero();
    auto turbine = seahowl::core::Turbine(turbine_elasto, turbine_aero);
    seahowl::io::populate_turbine_from_file((DATADIR / "IEA15MW/onshore/turbine.json").generic_string(), turbine);

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
    TestFrameworkDataset test_dataset({false, (ref_dir / "test_turbine_rpm_initial_pitch.csv").generic_string(),
                                       (test_dir / "test_turbine_rpm_initial_pitch.test.csv").generic_string()});
    test_dataset.test_csv.add_function("time (s)", [&system_elasto]() { return system_elasto.get_time(); });
    test_dataset.test_csv.add_function("rpm (-)", [&turbine]() { return turbine.rna.elasto.get_rpm(); });
    test_dataset.test_csv.add_function("axial torque (Nm)",
                                       [&turbine]() { return turbine.rna.elasto.get_axial_torque(); });

    while (time < 50.0) {
        // prestep
        // compute forces
        turbine.apply_control(time, dt);
        turbine.aero.compute_fluid_loads(env_model, time);
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

TEST_F(TestTurbine, rpm_initial_pitch_fpm) {
    // general options
    bool visualization_on = true;
    bool statics_prestep = true;
    // solver
    auto verbose = false;
    // timestepping
    double dt = 0.1;
    // wind
    auto wind_model = std::make_shared<seahowl::env::ConstantWind>();
    wind_model->set_wind_velocity(Vector3d(8.0, 0.0, 0.0));
    wind_model->shear_coefficient = 0.12;
    // env_model
    auto env_model = seahowl::env::EnvModel();
    env_model.addModel(wind_model);

    // turbine
    double initial_pitch = seahowl::PI / 8.0;

    // system
    auto system_elasto = SystemElastoChrono();
    system_elasto.set_gravitational_acceleration(Vector3d(0.0, 0.0, -9.81));
    auto system_chrono = system_elasto.chobj;

    // turbine
    auto turbine_elasto = seahowl::elasto::TurbineElasto();
    auto turbine_aero = seahowl::aero::TurbineAero();
    auto turbine = seahowl::core::Turbine(turbine_elasto, turbine_aero);
    seahowl::io::populate_turbine_from_file((DATADIR / "IEA15MW/onshore/turbine.json").generic_string(), turbine);
    // force FPM mode on blades
    for (auto& blade : turbine.elasto.rna.rotor->blades) {
        dynamic_cast<seahowl::elasto::BladeElastoFEA&>(*blade).fpm_mode = true;
    }

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
    TestFrameworkDataset test_dataset({false, (ref_dir / "test_turbine_rpm_initial_pitch_fpm.csv").generic_string(),
                                       (test_dir / "test_turbine_rpm_initial_pitch_fpm.test.csv").generic_string()});
    test_dataset.test_csv.add_function("time (s)", [&system_elasto]() { return system_elasto.get_time(); });
    test_dataset.test_csv.add_function("rpm (-)", [&turbine]() { return turbine.rna.elasto.get_rpm(); });

    while (time < 50.0) {
        // prestep
        // compute forces
        turbine.apply_control(time, dt);
        turbine.aero.compute_fluid_loads(env_model, time);
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

TEST_F(TestTurbine, rpm_initial_pitch_rigid_rotor) {
    // general options
    bool visualization_on = true;
    bool statics_prestep = true;
    // solver
    auto verbose = false;
    // timestepping
    double dt = 0.1;
    // wind
    auto wind_model = std::make_shared<seahowl::env::ConstantWind>();
    wind_model->set_wind_velocity(Vector3d(8.0, 0.0, 0.0));
    wind_model->shear_coefficient = 0.12;
    // env_model
    auto env_model = seahowl::env::EnvModel();
    env_model.addModel(wind_model);

    // turbine
    double initial_pitch = seahowl::PI / 8.0;

    // system
    auto system_elasto = SystemElastoChrono();
    system_elasto.set_gravitational_acceleration(Vector3d(0.0, 0.0, -9.81));
    auto system_chrono = system_elasto.chobj;

    // turbine
    auto turbine_elasto = seahowl::elasto::TurbineElasto();
    auto turbine_aero = seahowl::aero::TurbineAero();
    auto turbine = seahowl::core::Turbine(turbine_elasto, turbine_aero);
    seahowl::io::populate_turbine_from_file((DATADIR / "IEA15MW/onshore/turbine_rigid.json").generic_string(), turbine);

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
    TestFrameworkDataset test_dataset(
        {false, (ref_dir / "test_turbine_rpm_initial_pitch_rigid_rotor.csv").generic_string(),
         (test_dir / "test_turbine_rpm_initial_pitch_rigid_rotor.test.csv").generic_string()});
    test_dataset.test_csv.add_function("time (s)", [&system_elasto]() { return system_elasto.get_time(); });
    test_dataset.test_csv.add_function("rpm (-)", [&turbine]() { return turbine.rna.elasto.get_rpm(); });

    while (time < 50.0) {
        // prestep
        // compute forces
        turbine.apply_control(time, dt);
        turbine.aero.compute_fluid_loads(env_model, time);
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

TEST_F(TestTurbine, controller_target_rpm) {
    // general options
    bool visualization_on = true;
    bool statics_prestep = true;
    // solver
    auto verbose = false;
    // timestepping
    double dt = 0.05;
    // wind
    auto wind_model = std::make_shared<seahowl::env::ConstantWind>();
    wind_model->set_wind_velocity(Vector3d(8.0, 0.0, 0.0));
    wind_model->shear_coefficient = 0.12;
    // env_model
    auto env_model = seahowl::env::EnvModel();
    env_model.addModel(wind_model);
    // turbine
    double initial_pitch = seahowl::PI / 8.0;

    // system
    auto system_elasto = SystemElastoChrono();
    system_elasto.set_gravitational_acceleration(Vector3d(0.0, 0.0, -9.81));
    auto system_chrono = system_elasto.chobj;

    // turbine
    auto turbine_elasto = seahowl::elasto::TurbineElasto();
    auto turbine_aero = seahowl::aero::TurbineAero();
    auto turbine = seahowl::core::Turbine(turbine_elasto, turbine_aero);
    seahowl::io::populate_turbine_from_file((DATADIR / "IEA15MW/onshore/turbine_rigid.json").generic_string(), turbine);

    // make controller
    double target_rpm = 2.0;
    auto controller = std::make_shared<seahowl::servo::ControllerVariableTorque>();
    controller->target_rpm = target_rpm;
    turbine.controller = controller;
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
    TestFrameworkDataset test_dataset({false, (ref_dir / "test_turbine_controller_target_rpm.csv").generic_string(),
                                       (test_dir / "test_turbine_controller_target_rpm.test.csv").generic_string()});
    test_dataset.test_csv.add_function("time (s)", [&system_elasto]() { return system_elasto.get_time(); });
    test_dataset.test_csv.add_function("rpm (-)", [&turbine]() { return turbine.rna.elasto.get_rpm(); });

    while (time < 100) {
        // prestep
        // compute forces
        turbine.apply_control(time, dt);
        turbine.aero.compute_fluid_loads(env_model, time);
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

TEST_F(TestTurbine, actuator_disk) {
    // general options
    bool visualization_on = true;
    bool statics_prestep = true;
    // solver
    auto verbose = false;
    // timestepping
    double dt = 0.1;
    // wind
    auto wind_model = std::make_shared<seahowl::env::ConstantWind>();
    wind_model->set_wind_velocity(Vector3d(11.0, 0.0, 0.0));
    wind_model->shear_coefficient = 0.12;
    // env_model
    auto env_model = seahowl::env::EnvModel();
    env_model.addModel(wind_model);

    // turbine
    double initial_pitch = 0.0 * seahowl::PI / 1000.0;
    // power target
    auto reference_power = 15.5e6;

    // system
    auto system_elasto = SystemElastoChrono();
    system_elasto.set_gravitational_acceleration(Vector3d(0.0, 0.0, -9.81));
    auto system_chrono = system_elasto.chobj;

    // turbine
    auto turbine_elasto = seahowl::elasto::TurbineElasto();
    auto turbine_aero = seahowl::aero::TurbineAero();
    auto turbine = seahowl::core::Turbine(turbine_elasto, turbine_aero);
    seahowl::io::populate_turbine_from_file((DATADIR / "IEA15MW/onshore/turbine_disk.json").generic_string(), turbine);

    // remove controller
    double target_rpm = 7.56;
    auto controller = std::make_shared<seahowl::servo::ControllerVariableTorque>();
    controller->target_rpm = target_rpm;
    turbine.controller = controller;
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
    TestFrameworkDataset test_dataset({false, (ref_dir / "test_turbine_actuator_disk.csv").generic_string(),
                                       (test_dir / "test_turbine_actuator_disk.test.csv").generic_string()});

    test_dataset.test_csv.add_function("time (s)", [&system_elasto]() { return system_elasto.get_time(); });
    test_dataset.test_csv.add_function("rpm (-)", [&turbine]() { return turbine.rna.elasto.get_rpm(); });
    test_dataset.test_csv.add_function("power (W)", [&turbine]() { return turbine.get_generated_power(); });
    int count = 0;
    while (time < 100) {
        // prestep
        // compute forces
        turbine.apply_control(time, dt);
        turbine.aero.compute_fluid_loads(env_model, time);
        // prestep (accumulates loads from aero to elasto)
        turbine.prestep(time, dt);

        system_elasto.step(dt);
        time += dt;

        // poststep
        turbine.poststep(time, dt);
        if (count == 5) {
            test_dataset.test_csv.write_row();
            count = 0;
        }
        count += 1;
    }

    reference_power = 15.3e6;
    // wind
    wind_model->set_wind_velocity(Vector3d(15.0, 0.0, 0.0));
    // turbine
    initial_pitch = 11.0 * seahowl::PI / 180.0;
    turbine.rna.elasto.rotor->apply_collective_pitch_increment(initial_pitch);

    count = 0;
    while (time < 200) {
        // prestep
        // compute forces
        turbine.apply_control(time, dt);
        turbine.aero.compute_fluid_loads(env_model, time);
        // prestep (accumulates loads from aero to elasto)
        turbine.prestep(time, dt);

        system_elasto.step(dt);
        time += dt;

        // poststep
        turbine.poststep(time, dt);
        if (count == 5) {
            test_dataset.test_csv.write_row();
            count = 0;
        }
        count += 1;
    }

    EvaluateTest(test_dataset);
}

TEST_F(TestTurbine, multiturbines) {
    // general options
    bool visualization_on = true;
    bool statics_prestep = true;
    // solver
    auto verbose = false;
    // timestepping
    double dt = 0.1;
    // wind
    auto wind_model = std::make_shared<seahowl::env::ConstantWind>();
    wind_model->set_wind_velocity(Vector3d(8.0, 0.0, 0.0));
    wind_model->shear_coefficient = 0.12;
    // env_model
    auto env_model = std::make_shared<seahowl::env::EnvModel>();
    env_model->addModel(wind_model);

    // turbine
    double initial_pitch = seahowl::PI / 8.0;

    // system
    auto system_elasto = SystemElastoChrono();
    system_elasto.set_gravitational_acceleration(Vector3d(0.0, 0.0, -9.81));
    auto system_aero = seahowl::aero::SystemAero();

    // system core
    auto system_core = seahowl::core::System(system_elasto, system_aero);
    system_core.env_model = env_model;

    // turbines
    auto turbine_file = (DATADIR / "IEA15MW/onshore/turbine_rigid.json").generic_string();
    auto nturbines = 3;
    for (int ii = 0; ii < nturbines; ii++) {
        system_core.elasto.turbines.push_back(std::make_shared<seahowl::elasto::TurbineElasto>());
        system_core.aero.turbines.push_back(std::make_shared<seahowl::aero::TurbineAero>());
        system_core.turbines.push_back(std::make_shared<seahowl::core::Turbine>(*system_core.elasto.turbines.back(),
                                                                                *system_core.aero.turbines.back()));
        auto& turbine = *system_core.turbines.back();
        seahowl::io::populate_turbine_from_file(turbine_file, turbine);
        // empty controller
        turbine.controller = std::make_shared<seahowl::servo::Controller>();
        // translate
        turbine.build();
        turbine.elasto.translate(Vector3d(0.0 + ii * 150.0, 0.0 + ii * (-150.0), 0.0));
    }

    double time = 0.0;
    system_core.initialize(time, dt);

    // apply pitch before statics
    for (auto& turbine : system_core.turbines) {
        turbine->rna.elasto.rotor->apply_collective_pitch_increment(initial_pitch);
    }

    // statics
    if (statics_prestep) {
        system_elasto.do_statics(true, 10);
    }
    system_core.poststep(0.0, dt);  // to update positions aero after statics step

    // Setup TestFwDataSet
    TestFrameworkDataset test_dataset({false, (ref_dir / "test_turbine_multiturbines.csv").generic_string(),
                                       (test_dir / "test_turbine_multiturbines.test.csv").generic_string()});
    test_dataset.test_csv.add_function("time (s)", [&system_elasto]() { return system_elasto.get_time(); });
    // test_dataset.add_test_function("time (s)", system_elasto.get_time);
    for (size_t idx_turbine = 0; idx_turbine < system_core.turbines.size(); idx_turbine++)
        test_dataset.test_csv.add_function(
            "rpm turbine " + std::to_string(idx_turbine + 1) + " (-)",
            [&system_core, idx_turbine]() { return system_core.turbines[idx_turbine]->rna.elasto.get_rpm(); });

    while (time < 50.0) {
        // prestep
        system_core.prestep(time, dt);

        // step
        system_core.step(dt);
        time += dt;

        // poststep
        system_core.poststep(time, dt);
        test_dataset.test_csv.write_row();
    }

    EvaluateTest(test_dataset);
}
