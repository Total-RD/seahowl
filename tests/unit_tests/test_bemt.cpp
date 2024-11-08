#include "fixture_components.h"

#include <seahowl/elasto/chrono_adapters.h>
#include <seahowl/aero/bemt.h>
#include <seahowl/core/tower.h>
#include <seahowl/commons/numerics.h>
#include <seahowl/io/read_json.h>

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
    // Setup TestFwDataSet
    TestFrameworkDataset test_dataset({false,
                                       (ref_dir / "test_bemt_tower_shadow_check.csv").generic_string(),
                                       (test_dir / "test_bemt_tower_shadow_check.test.csv").generic_string(),
                                       {"x", "y", "z"}});

    // system
    auto system_elasto = SystemElastoChrono();
    system_elasto.set_gravitational_acceleration(Vector3d(0.0, 0.0, -9.81));

    // tower
    auto tower_elasto = seahowl::elasto::TowerElasto();
    auto tower_aero = seahowl::aero::TowerAero();
    auto tower = seahowl::core::Tower(tower_elasto, tower_aero);
    seahowl::io::populate_tower_from_json((DATADIR / "tower.csv").generic_string(), tower);

    // build elasto and aero parts
    tower.build();
    // assemble elasto part (add to elasto system)
    tower.elasto.assemble(system_elasto);
    // do a statics step to initialize all elasto variables
    tower.elasto.nodes.front()->set_fixed(true);
    system_elasto.do_statics(true, 0);
    // initialize tower to get elasto pos/rot into aero class
    tower.initialize(0.0, 0.1);

    auto wind_velocity = Vector3d(10.0, 3.0, 1.0);

    // position 1
    auto position1 = Vector3d(-14.0, -5.0, 50.0);
    Vector3d wind_velocity1 = wind_velocity;
    seahowl::aero::apply_tower_shadow_effect_on_wind(wind_velocity1, position1, tower_aero);

    test_dataset.add_row_data({wind_velocity1.x(), wind_velocity1.y(), wind_velocity1.z()});

    // position 2
    auto position2 = Vector3d(-15.0, 0.0, 45.0);
    Vector3d wind_velocity2 = wind_velocity;
    seahowl::aero::apply_tower_shadow_effect_on_wind(wind_velocity2, position2, tower_aero);

    test_dataset.add_row_data({wind_velocity2.x(), wind_velocity2.y(), wind_velocity2.z()});

    // position 3
    auto position3 = Vector3d(-16.0, 2.0, 20.0);
    Vector3d wind_velocity3 = wind_velocity;
    seahowl::aero::apply_tower_shadow_effect_on_wind(wind_velocity3, position3, tower_aero);

    test_dataset.add_row_data({wind_velocity3.x(), wind_velocity3.y(), wind_velocity3.z()});

    auto wind_velocity_xz = Vector3d(10.0, 0.0, 1.0);

    // position 4
    auto position4 = Vector3d(-6.0, 2.0, 20.0);
    Vector3d wind_velocity4 = wind_velocity_xz;
    seahowl::aero::apply_tower_shadow_effect_on_wind(wind_velocity4, position4, tower_aero);

    test_dataset.add_row_data({wind_velocity4.x(), wind_velocity4.y(), wind_velocity4.z()});

    // position 5
    auto position5 = Vector3d(-16.0, -2.0, 20.0);
    Vector3d wind_velocity5 = wind_velocity_xz;
    seahowl::aero::apply_tower_shadow_effect_on_wind(wind_velocity5, position5, tower_aero);

    test_dataset.add_row_data({wind_velocity5.x(), wind_velocity5.y(), wind_velocity5.z()});

    // position 5 wind rotation
    auto rot2 = AngleAxisd(PI / 36.0, Vector3d(0.0, 0.0, 1.0));
    auto position5_rot2 = rot2 * position5;
    Vector3d wind_velocity5_rot2 = rot2 * wind_velocity_xz;
    seahowl::aero::apply_tower_shadow_effect_on_wind(wind_velocity5_rot2, position5_rot2, tower_aero);
    auto rot2_wind = rot2.inverse() * wind_velocity5_rot2;

    test_dataset.add_row_data({rot2_wind.x(), rot2_wind.y(), rot2_wind.z()});

    // position 5 tower and wind rotation
    auto rot = AngleAxisd(PI / 36.0, Vector3d(0.0, 1.0, 0.0));
    auto position5_rot = rot * position5;
    for (auto& node : tower_aero.nodes) {
        node.set_position(rot * node.get_position());
    }
    Vector3d wind_velocity5_rot = rot * wind_velocity_xz;
    seahowl::aero::apply_tower_shadow_effect_on_wind(wind_velocity5_rot, position5_rot, tower_aero);
    auto rot_wind = rot.inverse() * wind_velocity5_rot;

    test_dataset.add_row_data({rot_wind.x(), rot_wind.y(), rot_wind.z()});

    EvaluateTest(test_dataset);
}
