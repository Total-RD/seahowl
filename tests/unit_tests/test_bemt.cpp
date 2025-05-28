#include "fixture_components.h"

#include <seahowl/elasto/chrono_adapters.h>
#include <seahowl/aero/bemt.h>
#include <seahowl/core/tower.h>
#include <seahowl/commons/numerics.h>
#include <seahowl/io/read_input.h>

#include <gtest/gtest.h>
#include <filesystem>  // C++17
using std::filesystem::path;

using namespace seahowl;
using namespace seahowl::elasto;

// The fixture for testing
class TestBEMT : public FixtureComponents {
  protected:
    TestBEMT() : FixtureComponents() {
        ref_dir /= "test_bemt/ref";
        test_dir /= "test_bemt/test";
    }
};

TEST_F(TestBEMT, tower_shadow_check) {
    Vector3d wind_velocity;

    // Setup TestFwDataSet
    TestFrameworkDataset test_dataset({false, (ref_dir / "test_bemt_tower_shadow_check.csv").generic_string(),
                                       (test_dir / "test_bemt_tower_shadow_check.test.csv").generic_string()});

    test_dataset.test_csv.add_function("wind velocity (m/s)", [&wind_velocity] { return wind_velocity; });
    // system
    auto system_elasto = SystemElastoChrono();
    system_elasto.set_gravitational_acceleration(Vector3d(0.0, 0.0, -9.81));

    // tower
    auto tower_elasto = std::make_shared<seahowl::elasto::TowerElasto>();
    auto tower_aero = std::make_shared<seahowl::aero::TowerAero>();
    auto tower = seahowl::core::Tower(tower_elasto, tower_aero);
    seahowl::io::populate_tower_from_file((DATADIR / "IEA15MW/onshore/tower.csv").generic_string(), tower);

    // build elasto and aero parts
    tower.build();
    // assemble elasto part (add to elasto system)
    tower.elasto.assemble(system_elasto);
    // do a statics step to initialize all elasto variables
    tower.elasto.nodes.front()->set_fixed(true);
    system_elasto.do_statics(true, 0);
    // initialize tower to get elasto pos/rot into aero class
    tower.initialize(0.0, 0.1);

    auto wind_velocity0 = Vector3d(10.0, 3.0, 1.0);

    // position 1
    auto position1 = Vector3d(-14.0, -5.0, 50.0);
    wind_velocity = wind_velocity0;
    seahowl::aero::apply_tower_shadow_effect_on_wind(wind_velocity, position1, *tower_aero);
    test_dataset.test_csv.write_row();

    // position 2
    auto position2 = Vector3d(-15.0, 0.0, 45.0);
    wind_velocity = wind_velocity0;
    seahowl::aero::apply_tower_shadow_effect_on_wind(wind_velocity, position2, *tower_aero);
    test_dataset.test_csv.write_row();

    // position 3
    auto position3 = Vector3d(-16.0, 2.0, 20.0);
    wind_velocity = wind_velocity0;
    seahowl::aero::apply_tower_shadow_effect_on_wind(wind_velocity, position3, *tower_aero);
    test_dataset.test_csv.write_row();

    auto wind_velocity_xz = Vector3d(10.0, 0.0, 1.0);

    // position 4
    auto position4 = Vector3d(-6.0, 2.0, 20.0);
    wind_velocity = wind_velocity_xz;
    seahowl::aero::apply_tower_shadow_effect_on_wind(wind_velocity, position4, *tower_aero);
    test_dataset.test_csv.write_row();

    // position 5
    auto position5 = Vector3d(-16.0, -2.0, 20.0);
    wind_velocity = wind_velocity_xz;
    seahowl::aero::apply_tower_shadow_effect_on_wind(wind_velocity, position5, *tower_aero);
    test_dataset.test_csv.write_row();

    // position 5 wind rotation
    auto rot2 = AngleAxisd(PI / 36.0, Vector3d(0.0, 0.0, 1.0));
    auto position5_rot2 = rot2 * position5;
    wind_velocity = rot2 * wind_velocity_xz;
    seahowl::aero::apply_tower_shadow_effect_on_wind(wind_velocity, position5_rot2, *tower_aero);
    wind_velocity = rot2.inverse() * wind_velocity;
    test_dataset.test_csv.write_row();

    // position 5 tower and wind rotation
    auto rot = AngleAxisd(PI / 36.0, Vector3d(0.0, 1.0, 0.0));
    auto position5_rot = rot * position5;
    for (auto& node : tower_aero->nodes) {
        node.set_position(rot * node.get_position());
    }
    wind_velocity = rot * wind_velocity_xz;
    seahowl::aero::apply_tower_shadow_effect_on_wind(wind_velocity, position5_rot, *tower_aero);
    wind_velocity = rot.inverse() * wind_velocity;
    test_dataset.test_csv.write_row();

    EvaluateTest(test_dataset);
}
