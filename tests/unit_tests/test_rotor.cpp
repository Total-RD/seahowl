#include <gtest/gtest.h>
#include <spdlog/spdlog.h>
#include <seahowl/elasto/chrono_adapters.h>
#include <seahowl/elasto/blade_elasto.h>

#include <seahowl/io/read_json.h>
#include "tools/testfw_dataset.hpp"
#include "fixture_components.h"

#include <filesystem>  // C++17
using std::filesystem::path;

using namespace seahowl;
using namespace seahowl::elasto;

// The fixture for testing
class Test_rotor : public Fixture_components {
  protected:
    Test_rotor() : Fixture_components() {
        ref_dir /= "test_rotor/ref";
        test_dir /= "test_rotor/test";
    }
};

TEST_F(Test_rotor, mass) {
    // Setup TestFwDataSet
    TestFwDataSet test_dataset({.debug = false,
                                .reference_filepath = ref_dir / "test_rotor_mass.values.csv",
                                .test_filepath = test_dir / "test_rotor_mass.values.test.csv",
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

    test_dataset.testAddRow({rna.get_mass()});

    evaluate_test(test_dataset);
}
