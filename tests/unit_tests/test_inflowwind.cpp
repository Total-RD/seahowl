#include "fixture_components.h"

#include <seahowl/commons/numerics.h>
#include <seahowl/aero/turbine_aero.h>
#include <seahowl/core/turbine.h>
#include <seahowl/servo/controller.h>
#include <seahowl/io/read_json.h>
#include <seahowl/env/inflowwind_adapter.h>
#include <seahowl/elasto/chrono_adapters.h>

#include <gtest/gtest.h>
#include <filesystem>  // C++17
using std::filesystem::path;

using namespace seahowl;
using namespace seahowl::elasto;

// The fixture for testing
class TestInflowWind : public FixtureComponents {
  protected:
    TestInflowWind() : FixtureComponents() {
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
    // wind
    auto wind_model =
        seahowl::env::InflowWindAdapter((DATADIR / "aerodyn/IEA-15-240-RWT_InflowWind.dat").generic_string());
    // turbine
    double initial_pitch = seahowl::PI / 8.0;

    // system
    auto system_elasto = SystemElastoChrono();
    system_elasto.set_gravitational_acceleration(Vector3d(0.0, 0.0, -9.81));

    // turbine
    auto turbine_elasto = seahowl::elasto::TurbineElasto();
    auto turbine_aero = seahowl::aero::TurbineAero();
    auto turbine = seahowl::core::Turbine(turbine_elasto, turbine_aero);
    populate_turbine_from_json((DATADIR / "turbine_nocontrol.json").generic_string(), turbine);
    // remove controller
    turbine.controller = std::make_shared<seahowl::servo::Controller>();

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
    TestFrameworkDataset test_dataset(
        {.debug = false,
         .reference_filepath = (ref_dir / "test_inflowwind_rpm_initial_pitch.csv").generic_string(),
         .test_filepath = (test_dir / "test_inflowwind_rpm_initial_pitch.test.csv").generic_string(),
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
        test_dataset.add_row();
    }

    EvaluateTest(test_dataset);
}
