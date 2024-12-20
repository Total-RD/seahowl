#include "fixture_components.h"

#include <gtest/gtest.h>
#include <spdlog/spdlog.h>
#include <seahowl/core/simulation.h>
#include <seahowl/env/wind_models.h>
#include <seahowl/core/turbine.h>
#include <seahowl/core/system.h>
#include <seahowl/elasto/blade_elasto.h>
#include <seahowl/io/read_json.h>

#include <filesystem>  // C++17
using std::filesystem::path;

using namespace seahowl;
using namespace seahowl::elasto;

// The fixture for testing
class TestController : public FixtureComponents {
  protected:
    TestController() : FixtureComponents() {
        ref_dir /= "test_controller/ref";
        test_dir /= "test_controller/test";
    }
};

TEST_F(TestController, IEA15) {
    // wind speeds to test at different times: pairs are (time, speed)
    std::vector<std::pair<double, double>> time_speed_vector;
    time_speed_vector.push_back(std::make_pair(0.0, 8.0));
    time_speed_vector.push_back(std::make_pair(150.0, 10.0));
    time_speed_vector.push_back(std::make_pair(200.0, 12.0));
    time_speed_vector.push_back(std::make_pair(250.0, 15.0));
    time_speed_vector.push_back(std::make_pair(300.0, 20.0));
    time_speed_vector.push_back(std::make_pair(350.0, 25.0));

    // make simulation object
    auto simulation = seahowl::core::Simulation();
    simulation.dt = 0.05;
    simulation.duration = 400.0;
    simulation.outputs->dt_output = 9999.9;  // no output, using custom CSV
    simulation.outputs->has_csv = false;
    simulation.outputs->has_gui = false;
    simulation.outputs->has_vtk = false;

    auto& system_core = *simulation.system_core;
    auto& system_elasto = system_core.elasto;

    // add turbine to system
    seahowl::io::add_turbine_to_system_from_json((DATADIR / "IEA15MW/turbine.json").generic_string(), system_core,
                                                 "./output");
    auto& turbine = *system_core.turbines[0];
    // fix tower bottom
    turbine.elasto.tower.nodes.front()->set_fixed(true);
    // statics
    system_elasto.do_statics(true, 10);

    // add wind model
    auto wind_model = std::make_shared<seahowl::env::ConstantWind>();
    system_core.fluid_model = wind_model;
    wind_model->shear_coefficient = 0.12;
    wind_model->reference_height = turbine.elasto.rna.rotor->body_hub->get_position().z();
    wind_model->set_wind_velocity(seahowl::Vector3d(time_speed_vector[0].second, 0.0, 0.0));

    // initialize simulation
    simulation.initialize();

    // instantiate test dataset class (custom CSV)
    TestFrameworkDataset test_dataset({false, (ref_dir / "test_controller.csv").generic_string(),
                                       (test_dir / "test_controller.test.csv").generic_string()});
    test_dataset.test_csv.add_function(
        "time (s)", [&system_elasto]() -> std::vector<double> { return {system_elasto.get_time()}; });
    test_dataset.test_csv.add_function("power (W)",
                                       [&turbine]() -> std::vector<double> { return {turbine.get_generated_power()}; });
    for (size_t idx_blade = 0; idx_blade < turbine.rna.elasto.rotor->blades.size(); idx_blade++) {
        test_dataset.test_csv.add_function("pitch blade" + std::to_string(idx_blade + 1) + " (rad)",
                                           [&turbine, idx_blade]() -> std::vector<double> {
                                               return {turbine.rna.elasto.rotor->blades[idx_blade]->get_pitch()};
                                           });
    }

    // output values at time = 0
    test_dataset.test_csv.write_row();

    // simulation loop
    size_t istep = 0;
    while (system_core.get_time() < simulation.duration) {
        // check if new wind speed needs to be set
        for (auto& time_speed_pair : time_speed_vector) {
            if (istep == (int)round(time_speed_pair.first / simulation.dt)) {
                auto wind_speed = time_speed_pair.second;
                spdlog::info("Setting wind speed to {}m/s after {:.2f}s simulation time.", wind_speed,
                             round(system_core.get_time()));
                wind_model->set_wind_velocity(seahowl::Vector3d(wind_speed, 0.0, 0.0));
            }
        }

        // simulation step
        simulation.step();
        istep += 1;

        // check if time to store results
        if ((int)round(1.0 / simulation.dt)) {
            test_dataset.test_csv.write_row();
        }
    }

    // compare ref and test CSVs
    EvaluateTest(test_dataset);
}

TEST_F(TestController, actuator_disk) {
    // general options
    bool visualization_on = true;
    bool statics_prestep = true;
    // solver
    auto verbose = false;
    // timestepping
    double dt = 0.05;
    double simul_time = 350.0;
    // wind ramp
    auto wind_model = seahowl::env::WindRamp();
    const Vector3d wind_in(5.0, 0.0, 0.0);
    const Vector3d wind_end(15.0, 0.0, 0.0);
    wind_model.set_wind_ramp(wind_in, 0.0, wind_end, 250.0);

    wind_model.shear_coefficient = 0.12;
    // turbine
    double initial_pitch = 0.0 * seahowl::PI / 1000.0;

    // system
    auto system_elasto = SystemElastoChrono();
    auto system_chrono = system_elasto.chobj;

    // turbine
    auto turbine_elasto = seahowl::elasto::TurbineElasto();
    auto turbine_aero = seahowl::aero::TurbineAero();
    auto turbine = seahowl::core::Turbine(turbine_elasto, turbine_aero);
    seahowl::io::populate_turbine_from_json((DATADIR / "IEA15MW/turbine_disk.json").generic_string(), turbine);

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
    TestFrameworkDataset test_dataset({false, (ref_dir / "test_controller_actuator_disk.csv").generic_string(),
                                       (test_dir / "test_controller_actuator_disk.test.csv").generic_string()});

    test_dataset.test_csv.add_function(
        "time (s)", [&system_elasto]() -> std::vector<double> { return {system_elasto.get_time()}; });

    test_dataset.test_csv.add_function("rpm (-)",
                                       [&turbine]() -> std::vector<double> { return {turbine.rna.elasto.get_rpm()}; });
    test_dataset.test_csv.add_function("power (W)",
                                       [&turbine]() -> std::vector<double> { return {turbine.get_generated_power()}; });

    int count = 0;
    while (time < simul_time) {
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
        if (count == 10) {
            test_dataset.test_csv.write_row();
            count = 0;
        }
        count += 1;
    }

    EvaluateTest(test_dataset);
}
