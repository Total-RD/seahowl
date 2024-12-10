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
    TestFrameworkDataset test_dataset(
        {false,
         (ref_dir / "test_controller.csv").generic_string(),
         (test_dir / "test_controller.test.csv").generic_string(),
         {"time", "power", "pitch_blade1", "pitch_blade2", "pitch_blade3"},
         {
             [&system_elasto]() -> std::vector<double> { return {system_elasto.get_time()}; },
             [&turbine]() -> std::vector<double> { return {turbine.get_generated_power()}; },
             [&turbine]() -> std::vector<double> { return {turbine.rna.elasto.rotor->blades[0]->get_pitch()}; },
             [&turbine]() -> std::vector<double> { return {turbine.rna.elasto.rotor->blades[1]->get_pitch()}; },
             [&turbine]() -> std::vector<double> { return {turbine.rna.elasto.rotor->blades[2]->get_pitch()}; },
         }});

    // output values at time = 0
    test_dataset.add_row();

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
            test_dataset.add_row();
        }
    }

    // compare ref and test CSVs
    EvaluateTest(test_dataset);
}
