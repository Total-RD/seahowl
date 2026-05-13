// SPDX-License-Identifier: Apache-2.0
// Copyright 2022–2026 TotalEnergies
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//

// Local test headers
#include "fixture_components.h"

// SEAHOWL headers
#include <seahowl/elasto.h>
#include <seahowl/io.h>

// Third-party libraries
#include <gtest/gtest.h>
#include <spdlog/spdlog.h>

// Standard library
#include <filesystem>

using std::filesystem::path;

using namespace seahowl;
using namespace seahowl::elasto;

// The fixture for testing
class TestRotor : public FixtureComponents {
  protected:
    TestRotor() : FixtureComponents() {
        root_dir /= "test_rotor";
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
        seahowl::io::populate_blade_elasto_from_file((DATADIR / "IEA15MW/base/blade.json").generic_string(),
                                                     *blade.get());
        // make 50 elements
        blade->discretization_fractions.clear();
        for (int ii = 0; ii < 51; ii++) {
            blade->discretization_fractions.push_back(0.02 * ii);
        }
        blades.push_back(blade);
    }
    auto rotor = std::make_shared<RotorElasto>();
    auto rna = seahowl::elasto::RotorNacelleAssemblyElasto(rotor);
    seahowl::io::populate_rna_elasto_from_file((DATADIR / "IEA15MW/base/rna.json").generic_string(), rna);
    rna.rotor->blades = blades;
    rna.build();
    rna.assemble(system_elasto);
    rna.actuator_yaw->body_controller->set_fixed(true);

    system_elasto.do_statics(true, 0);

    // test mass
    TestFrameworkDataset test_dataset_mass({false, (ref_dir / "test_rotor_mass.csv").generic_string(),
                                            (test_dir / "test_rotor_mass.test.csv").generic_string()});
    test_dataset_mass.test_csv.add_function("mass [kg]", [&rna] { return rna.get_mass(); });
    test_dataset_mass.test_csv.write_row();
    EvaluateTest(test_dataset_mass);
}
