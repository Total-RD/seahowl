#include "fixture_components.h"

#include <seahowl/elasto/chrono_adapters.h>
#include <seahowl/elasto/blade_elasto.h>
#include <seahowl/io/read_json.h>

#include <gtest/gtest.h>
#include <spdlog/spdlog.h>
#include <filesystem>  // C++17
using std::filesystem::path;

using namespace seahowl;
using namespace seahowl::elasto;

// The fixture for testing
class TestRotor : public FixtureComponents {
  protected:
    TestRotor() : FixtureComponents() {
        ref_dir /= "test_rotor/ref";
        test_dir /= "test_rotor/test";
    }
};

TEST_F(TestRotor, mass) {
    // Setup TestFwDataSet
    TestFrameworkDataset test_dataset({.debug = false,
                                       .reference_filepath = (ref_dir / "test_rotor_mass.values.csv").generic_string(),
                                       .test_filepath = (test_dir / "test_rotor_mass.values.test.csv").generic_string(),
                                       .dimensions = {"values"}});

    // system
    auto system_elasto = SystemElastoChrono();

    std::vector<std::shared_ptr<seahowl::elasto::BladeElasto>> blades;
    for (int ii = 0; ii < 3; ii++) {
        auto blade = std::make_shared<seahowl::elasto::BladeElastoFEA>();
        populate_blade_elasto_from_json((DATADIR / "blade.json").generic_string(), *blade.get());
        // make 50 elements
        blade->discretization_fractions.clear();
        for (int ii = 0; ii < 51; ii++) {
            blade->discretization_fractions.push_back(0.02 * ii);
        }
        blades.push_back(blade);
    }

    auto rna = seahowl::elasto::RotorNacelleAssemblyElasto();
    populate_rna_elasto_from_json((DATADIR / "rna.json").generic_string(), rna);
    rna.rotor->blades = blades;
    rna.build();
    rna.assemble(system_elasto);
    rna.body_yaw_bearing->set_fixed(true);

    system_elasto.do_statics(true, 0);

    test_dataset.add_row_data({rna.get_mass()});

    EvaluateTest(test_dataset);
}
