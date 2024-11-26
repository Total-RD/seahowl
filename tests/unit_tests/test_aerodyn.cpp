#include "fixture_components.h"

#include <seahowl/env/wind_models.h>
#include <seahowl/commons/numerics.h>
#include <seahowl/elasto/chrono_adapters.h>
#include <seahowl/core/turbine.h>
#include <seahowl/servo/controller.h>
#include <seahowl/io/read_json.h>
#include <seahowl/aero/aerodyn_adapter.h>
#include <seahowl/env/inflowwind_adapter.h>
using namespace seahowl;
using namespace seahowl::elasto;

#include <gtest/gtest.h>
#include <memory>
#include <filesystem>  // C++17
using std::filesystem::path;

// The fixture for testing
class TestAeroDyn : public FixtureComponents {
  protected:
    TestAeroDyn() : FixtureComponents() {
        ref_dir /= "test_aerodyn/ref";
        test_dir /= "test_aerodyn/test";
    }
};

TEST_F(TestAeroDyn, rpm_initial_pitch) {
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
    auto turbine_aero = seahowl::aero::TurbineAeroDyn();
    auto turbine = seahowl::core::Turbine(turbine_elasto, turbine_aero);
    seahowl::io::populate_turbine_from_json((DATADIR / "turbine_nocontrol_aerodyn.json").generic_string(), turbine);
    // remove controller
    turbine.controller = std::make_shared<seahowl::servo::Controller>();

    turbine_aero.aerodyn.set_infiles((DATADIR / "aerodyn/IEA-15-240-RWT_AeroDyn15.dat").generic_string(),
                                     (DATADIR / "aerodyn/IEA-15-240-RWT_InflowWind.dat").generic_string());

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
    TestFrameworkDataset test_dataset({false, (ref_dir / "test_aerodyn_rpm_initial_pitch.csv").generic_string(),
                                       (test_dir / "test_aerodyn_rpm_initial_pitch.test.csv").generic_string()});
    test_dataset.test_csv.add_function(
        "time (s)", [&system_elasto]() -> std::vector<double> { return {system_elasto.get_time()}; });
    test_dataset.test_csv.add_function("rpm (-)",
                                       [&turbine]() -> std::vector<double> { return {turbine.rna.elasto.get_rpm()}; });

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
        test_dataset.test_csv.write_row();
    }

    EvaluateTest(test_dataset);
}
