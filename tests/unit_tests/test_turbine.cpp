#include <gtest/gtest.h>
#include <spdlog/spdlog.h>
#include <seahowl/elasto/chrono_adapters.h>
#include <seahowl/env/wind_models.h>
#include <seahowl/commons/numerics.h>
#include <seahowl/aero/turbine_aero.h>
#include <seahowl/core/turbine.h>
#include <seahowl/core/system.h>
#include <seahowl/servo/controller.h>
#include <seahowl/aero/system_aero.h>
#include "tools/testfw_dataset.hpp"
#include "fixture_components.h"

#include <seahowl/io/read_json.h>

#include <filesystem>  // C++17
using std::filesystem::path;

using namespace seahowl;
using namespace seahowl::elasto;

// The fixture for testing
class Test_turbine : public Fixture_components {
  protected:
    Test_turbine() : Fixture_components() {
        ref_dir /= "test_turbine/ref";
        test_dir /= "test_turbine/test";
    }
};

TEST_F(Test_turbine, rpm_initial_pitch) {
    // general options
    bool visualization_on = true;
    bool statics_prestep = true;
    // solver
    auto verbose = false;
    // timestepping
    double dt = 0.1;
    // wind
    auto wind_model = seahowl::env::ConstantWind();
    wind_model.set_wind_velocity(Vector3d(8.0, 0.0, 0.0));
    wind_model.shear_coefficient = 0.12;
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
    populate_turbine_from_json((DATADIR / "turbine_nocontrol.json").generic_string(), turbine);

    // remove controller
    turbine.controller = std::make_shared<seahowl::servo::Controller>();
    turbine.build();
    turbine.elasto.assemble(system_elasto);
    turbine.tower.elasto.nodes.front()->set_fixed(true);

    // statics
    if (statics_prestep) {
        turbine.rna.elasto.link_shaft_hub->set_constraints(true, true, true, true, true, true);
        system_elasto.do_statics(true, 10);
        turbine.rna.elasto.link_shaft_hub->set_constraints(true, true, true, false, true, true);
    }

    double time = 0.0;
    turbine.rna.elasto.rotor->apply_collective_pitch_increment(initial_pitch);
    turbine.initialize(time, dt);

    // Setup TestFwDataSet
    TestFwDataSet test_dataset(
        {.debug = false,
         .reference_filepath = (ref_dir / "test_turbine_rpm_initial_pitch.csv").generic_string(),
         .test_filepath = (test_dir / "test_turbine_rpm_initial_pitch.test.csv").generic_string(),
         .dimensions = {"time", "rpm", "axial_torque"},
         .test_functions = {[&system_elasto]() -> std::vector<double> { return {system_elasto.get_time()}; },
                            [&turbine]() -> std::vector<double> { return {turbine.rna.elasto.get_rpm()}; },
                            [&turbine]() -> std::vector<double> { return {turbine.rna.elasto.get_axial_torque()}; }}

        });

    while (time < 50) {
        // prestep
        // compute forces
        turbine.apply_control(time, dt);
        turbine.aero.compute_fluid_loads(wind_model, time);
        // prestep (accumulates loads from aero to elasto)
        turbine.prestep(time, dt);

        system_elasto.step(dt);
        time += dt;

        // poststep
        turbine.poststep(time, dt);
        test_dataset.testAdd();
    }

    ASSERT_NEAR(turbine.rna.elasto.get_rpm(), 2.687975, 1e-4);
    evaluate_test(test_dataset);
}

TEST_F(Test_turbine, rpm_initial_pitch_fpm) {
    // general options
    bool visualization_on = true;
    bool statics_prestep = true;
    // solver
    auto verbose = false;
    // timestepping
    double dt = 0.1;
    // wind
    auto wind_model = seahowl::env::ConstantWind();
    wind_model.set_wind_velocity(Vector3d(8.0, 0.0, 0.0));
    wind_model.shear_coefficient = 0.12;
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
    populate_turbine_from_json((DATADIR / "turbine_nocontrol_fpm.json").generic_string(), turbine);

    // remove controller
    turbine.controller = std::make_shared<seahowl::servo::Controller>();
    turbine.build();
    turbine.elasto.assemble(system_elasto);
    turbine.tower.elasto.nodes.front()->set_fixed(true);

    // statics
    if (statics_prestep) {
        turbine.rna.elasto.link_shaft_hub->set_constraints(true, true, true, true, true, true);
        system_elasto.do_statics(true, 10);
        turbine.rna.elasto.link_shaft_hub->set_constraints(true, true, true, false, true, true);
    }

    double time = 0.0;
    turbine.rna.elasto.rotor->apply_collective_pitch_increment(initial_pitch);
    turbine.initialize(time, dt);

    // Setup TestFwDataSet
    TestFwDataSet test_dataset(
        {.debug = false,
         .reference_filepath = (ref_dir / "test_turbine_rpm_initial_pitch_fpm.csv").generic_string(),
         .test_filepath = (test_dir / "test_turbine_rpm_initial_pitch_fpm.test.csv").generic_string(),
         .dimensions = {"time", "rpm"},
         .test_functions = {[&system_elasto]() -> std::vector<double> { return {system_elasto.get_time()}; },
                            [&turbine]() -> std::vector<double> { return {turbine.rna.elasto.get_rpm()}; }}});

    while (time < 50) {
        // prestep
        // compute forces
        turbine.apply_control(time, dt);
        turbine.aero.compute_fluid_loads(wind_model, time);
        // prestep (accumulates loads from aero to elasto)
        turbine.prestep(time, dt);

        system_elasto.step(dt);
        time += dt;

        // poststep
        turbine.poststep(time, dt);
        test_dataset.testAdd();
    }

    ASSERT_NEAR(turbine.rna.elasto.get_rpm(), 2.691299, 1e-4);

    evaluate_test(test_dataset);
}

TEST_F(Test_turbine, rpm_initial_pitch_rigid_rotor) {
    // general options
    bool visualization_on = true;
    bool statics_prestep = true;
    // solver
    auto verbose = false;
    // timestepping
    double dt = 0.1;
    // wind
    auto wind_model = seahowl::env::ConstantWind();
    wind_model.set_wind_velocity(Vector3d(8.0, 0.0, 0.0));
    wind_model.shear_coefficient = 0.12;
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
    populate_turbine_from_json((DATADIR / "turbine_nocontrol_rigid.json").generic_string(), turbine);

    // remove controller
    turbine.controller = std::make_shared<seahowl::servo::Controller>();
    turbine.build();
    turbine.elasto.assemble(system_elasto);
    turbine.tower.elasto.nodes.front()->set_fixed(true);

    // statics
    if (statics_prestep) {
        turbine.rna.elasto.link_shaft_hub->set_constraints(true, true, true, true, true, true);
        system_elasto.do_statics(true, 10);
        turbine.rna.elasto.link_shaft_hub->set_constraints(true, true, true, false, true, true);
    }

    double time = 0.0;
    turbine.rna.elasto.rotor->apply_collective_pitch_increment(initial_pitch);
    turbine.initialize(time, dt);

    // Setup TestFwDataSet
    TestFwDataSet test_dataset(
        {.debug = false,
         .reference_filepath = (ref_dir / "test_turbine_rpm_initial_pitch_rigid_rotor.csv").generic_string(),
         .test_filepath = (test_dir / "test_turbine_rpm_initial_pitch_rigid_rotor.test.csv").generic_string(),
         .dimensions = {"time", "rpm"},
         .test_functions = {[&system_elasto]() -> std::vector<double> { return {system_elasto.get_time()}; },
                            [&turbine]() -> std::vector<double> { return {turbine.rna.elasto.get_rpm()}; }}});

    while (time < 50) {
        // prestep
        // compute forces
        turbine.apply_control(time, dt);
        turbine.aero.compute_fluid_loads(wind_model, time);
        // prestep (accumulates loads from aero to elasto)
        turbine.prestep(time, dt);

        system_elasto.step(dt);
        time += dt;

        // poststep
        turbine.poststep(time, dt);
        test_dataset.testAdd();
    }

    evaluate_test(test_dataset);
    ASSERT_NEAR(turbine.rna.elasto.get_rpm(), 2.709388, 1e-4);
}

TEST_F(Test_turbine, controller_target_rpm) {
    // general options
    bool visualization_on = true;
    bool statics_prestep = true;
    // solver
    auto verbose = false;
    // timestepping
    double dt = 0.05;
    // wind
    auto wind_model = seahowl::env::ConstantWind();
    wind_model.set_wind_velocity(Vector3d(8.0, 0.0, 0.0));
    wind_model.shear_coefficient = 0.12;
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
    populate_turbine_from_json((DATADIR / "turbine_nocontrol_rigid.json").generic_string(), turbine);

    // remove controller
    double target_rpm = 2.0;
    auto controller = std::make_shared<seahowl::servo::ControllerVariableTorque>();
    controller->target_rpm = target_rpm;
    turbine.controller = controller;
    turbine.build();
    turbine.elasto.assemble(system_elasto);
    turbine.tower.elasto.nodes.front()->set_fixed(true);

    // statics
    if (statics_prestep) {
        turbine.rna.elasto.link_shaft_hub->set_constraints(true, true, true, true, true, true);
        system_elasto.do_statics(true, 10);
        turbine.rna.elasto.link_shaft_hub->set_constraints(true, true, true, false, true, true);
    }

    double time = 0.0;
    turbine.rna.elasto.rotor->apply_collective_pitch_increment(initial_pitch);
    turbine.initialize(time, dt);

    // Setup TestFwDataSet
    TestFwDataSet test_dataset(
        {.debug = false,
         .reference_filepath = (ref_dir / "test_turbine_controller_target_rpm.csv").generic_string(),
         .test_filepath = (test_dir / "test_turbine_controller_target_rpm.test.csv").generic_string(),
         .dimensions = {"time", "rpm"},
         .test_functions = {[&system_elasto]() -> std::vector<double> { return {system_elasto.get_time()}; },
                            [&turbine]() -> std::vector<double> { return {turbine.rna.elasto.get_rpm()}; }}});

    while (time < 100) {
        // prestep
        // compute forces
        turbine.apply_control(time, dt);
        turbine.aero.compute_fluid_loads(wind_model, time);
        // prestep (accumulates loads from aero to elasto)
        turbine.prestep(time, dt);

        system_elasto.step(dt);
        time += dt;

        // poststep
        turbine.poststep(time, dt);
        test_dataset.testAdd();
    }

    evaluate_test(test_dataset);
    ASSERT_NEAR(turbine.rna.elasto.get_rpm(), target_rpm, 1e-2);
}

TEST_F(Test_turbine, actuator_disk) {
    // general options
    bool visualization_on = true;
    bool statics_prestep = true;
    // solver
    auto verbose = false;
    // timestepping
    double dt = 0.1;
    // wind
    auto wind_model = seahowl::env::ConstantWind();
    wind_model.set_wind_velocity(Vector3d(11.0, 0.0, 0.0));
    wind_model.shear_coefficient = 0.12;
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
    populate_turbine_from_json((DATADIR / "turbine_disk.json").generic_string(), turbine);

    // remove controller
    double target_rpm = 7.56;
    auto controller = std::make_shared<seahowl::servo::ControllerVariableTorque>();
    controller->target_rpm = target_rpm;
    turbine.controller = controller;
    turbine.build();
    turbine.elasto.assemble(system_elasto);
    turbine.tower.elasto.nodes.front()->set_fixed(true);

    // statics
    if (statics_prestep) {
        turbine.rna.elasto.link_shaft_hub->set_constraints(true, true, true, true, true, true);
        system_elasto.do_statics(true, 10);
        turbine.rna.elasto.link_shaft_hub->set_constraints(true, true, true, false, true, true);
    }

    double time = 0.0;
    turbine.rna.elasto.rotor->apply_collective_pitch_increment(initial_pitch);
    turbine.initialize(time, dt);

    // Setup TestFwDataSet
    TestFwDataSet test_dataset(
        {.debug = false,
         .reference_filepath = (ref_dir / "test_turbine_actuator_disk.csv").generic_string(),
         .test_filepath = (test_dir / "test_turbine_actuator_disk.test.csv").generic_string(),
         .dimensions = {"time", "rpm", "generated_power"},
         .test_functions = {[&system_elasto]() -> std::vector<double> { return {system_elasto.get_time()}; },
                            [&turbine]() -> std::vector<double> { return {turbine.rna.elasto.get_rpm()}; },
                            [&turbine]() -> std::vector<double> { return {turbine.get_generated_power()}; }}});

    while (time < 100) {
        // prestep
        // compute forces
        turbine.apply_control(time, dt);
        turbine.aero.compute_fluid_loads(wind_model, time);
        // prestep (accumulates loads from aero to elasto)
        turbine.prestep(time, dt);

        system_elasto.step(dt);
        time += dt;

        // poststep
        turbine.poststep(time, dt);
        test_dataset.testAdd();
    }
    // ASSERT_NEAR(turbine.rna.elasto.get_rpm(), target_rpm, 1e-3);
    ASSERT_NEAR(turbine.get_generated_power(), reference_power, reference_power * 0.01);
    ASSERT_NEAR(turbine.rna.elasto.get_rpm(), target_rpm, target_rpm * 0.01);

    reference_power = 15.3e6;
    // wind
    wind_model.set_wind_velocity(Vector3d(15.0, 0.0, 0.0));
    // turbine
    initial_pitch = 11.0 * seahowl::PI / 180.0;
    turbine.rna.elasto.rotor->apply_collective_pitch_increment(initial_pitch);
    while (time < 200) {
        // prestep
        // compute forces
        turbine.apply_control(time, dt);
        turbine.aero.compute_fluid_loads(wind_model, time);
        // prestep (accumulates loads from aero to elasto)
        turbine.prestep(time, dt);

        system_elasto.step(dt);
        time += dt;

        // poststep
        turbine.poststep(time, dt);
        test_dataset.testAdd();
    }

    evaluate_test(test_dataset);
    ASSERT_NEAR(turbine.get_generated_power(), reference_power, reference_power * 0.01);
    ASSERT_NEAR(turbine.rna.elasto.get_rpm(), target_rpm, target_rpm * 0.01);
}

TEST_F(Test_turbine, multiturbines) {
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
    // turbine
    double initial_pitch = seahowl::PI / 8.0;

    // system
    auto system_elasto = SystemElastoChrono();
    system_elasto.set_gravitational_acceleration(Vector3d(0.0, 0.0, -9.81));
    auto system_aero = seahowl::aero::SystemAero();

    // system core
    auto system_core = seahowl::core::System(system_elasto, system_aero);
    system_core.fluid_model = wind_model;

    // turbines
    auto turbine_file = (DATADIR / "turbine_nocontrol_rigid.json").generic_string();
    auto nturbines = 3;
    for (int ii = 0; ii < nturbines; ii++) {
        system_core.elasto.turbines.push_back(std::make_shared<seahowl::elasto::TurbineElasto>());
        system_core.aero.turbines.push_back(std::make_shared<seahowl::aero::TurbineAero>());
        system_core.turbines.push_back(std::make_shared<seahowl::core::Turbine>(*system_core.elasto.turbines.back(),
                                                                                *system_core.aero.turbines.back()));
        auto& turbine = *system_core.turbines.back();
        populate_turbine_from_json(turbine_file, turbine);
        // empty controller
        turbine.controller = std::make_shared<seahowl::servo::Controller>();
        // translate
        turbine.build();
        turbine.elasto.translate(Vector3d(0.0 + ii * 150.0, 0.0 + ii * (-150.0), 0.0));
        // fix
        turbine.tower.elasto.nodes.front()->set_fixed(true);
    }

    // assemble system
    system_core.elasto.assemble();

    // statics
    if (statics_prestep) {
        system_elasto.do_statics(true, 10);
    }

    double time = 0.0;
    for (auto& turbine : system_core.turbines) {
        turbine->rna.elasto.rotor->apply_collective_pitch_increment(initial_pitch);
    }
    system_core.initialize(time, dt);

    // Setup TestFwDataSet
    TestFwDataSet test_dataset(
        {.debug = false,
         .reference_filepath = (ref_dir / "test_turbine_multiturbines.csv").generic_string(),
         .test_filepath = (test_dir / "test_turbine_multiturbines.test.csv").generic_string(),
         .dimensions = {"time", "rpm turb0", "rpm turb1", "rpm turb2"},
         .test_functions = {
             [&system_elasto]() -> std::vector<double> { return {system_elasto.get_time()}; },
             [&system_core]() -> std::vector<double> { return {system_core.turbines[0]->rna.elasto.get_rpm()}; },
             [&system_core]() -> std::vector<double> { return {system_core.turbines[1]->rna.elasto.get_rpm()}; },
             [&system_core]() -> std::vector<double> { return {system_core.turbines[2]->rna.elasto.get_rpm()}; }}});

    while (time < 50) {
        // prestep
        system_core.prestep(time, dt);

        // step
        system_core.step(dt);
        time += dt;

        // poststep
        system_core.poststep(time, dt);
        test_dataset.testAdd();
    }

    for (auto& turbine : system_core.turbines) {
        ASSERT_NEAR(turbine->rna.elasto.get_rpm(), 2.728150, 0.02);
    }
    evaluate_test(test_dataset);
}
