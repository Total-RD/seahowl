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
    // system
    auto system_elasto = SystemElastoChrono();

    std::vector<std::shared_ptr<seahowl::elasto::BladeElasto>> blades;
    for (int ii = 0; ii < 3; ii++) {
        auto blade = std::make_shared<seahowl::elasto::BladeElastoFEA>();
        seahowl::io::populate_blade_elasto_from_json((DATADIR / "IEA15MW/base/blade.json").generic_string(),
                                                     *blade.get());
        // make 50 elements
        blade->discretization_fractions.clear();
        for (int ii = 0; ii < 51; ii++) {
            blade->discretization_fractions.push_back(0.02 * ii);
        }
        blades.push_back(blade);
    }

    auto rna = seahowl::elasto::RotorNacelleAssemblyElasto();
    seahowl::io::populate_rna_elasto_from_json((DATADIR / "IEA15MW/base/rna.json").generic_string(), rna);
    rna.rotor->blades = blades;
    rna.build();
    rna.assemble(system_elasto);
    rna.body_yaw_bearing->set_fixed(true);

    system_elasto.do_statics(true, 0);

    // test mass
    TestFrameworkDataset test_dataset_mass({false, (ref_dir / "test_rotor_mass.csv").generic_string(),
                                            (test_dir / "test_rotor_mass.test.csv").generic_string()});
    test_dataset_mass.test_csv.add_function("mass (kg)", [&rna] { return rna.get_mass(); });
    test_dataset_mass.test_csv.write_row();
    EvaluateTest(test_dataset_mass);
}
