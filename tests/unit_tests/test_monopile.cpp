// SPDX-License-Identifier: Apache-2.0
// Copyright 2022–2026 TotalEnergies
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//

#include "fixture_components.h"

#include <gtest/gtest.h>
#include <spdlog/spdlog.h>
#include <seahowl/elasto/chrono_adapters.h>
#include <seahowl/core/monopile.h>
#include <seahowl/elasto/monopile_elasto.h>
#include <seahowl/env/wind_models.h>
#include <seahowl/env/env_model.h>
#include <seahowl/commons/numerics.h>
#include <seahowl/fluid/turbine_fluid.h>
#include <seahowl/core/turbine.h>
#include <seahowl/core/system.h>
#include <seahowl/servo/controller.h>
#include <seahowl/fluid/system_fluid.h>
#include <seahowl/io/read_input.h>
#include <seahowl/io/input_structures.h>
#include <seahowl/io/input_handler.h>

#ifdef HAVE_HYDROCHRONO
    #include <seahowl/fluid/hydro/hydrochrono_adapter.h>
    #include <hydroc/hydro_forces.h>
#endif
#ifdef HAVE_HYDRODYN
    #include <seahowl/fluid/hydro/hydrodyn_adapter.h>
#endif
#ifdef HAVE_SEASTATE
    #include <seahowl/env/seastate_adapter.h>
#endif

#include <filesystem>  // C++17
using std::filesystem::path;

using namespace seahowl;
using namespace seahowl::elasto;

// The fixture for testing
class TestMonopile : public FixtureComponents {
  protected:
    TestMonopile() : FixtureComponents() {
        root_dir /= "test_monopile";
        ref_dir /= "test_monopile/ref";
        test_dir /= "test_monopile/test";
    }
};

#ifdef HAVE_HYDROCHRONO
TEST_F(TestMonopile, monopile_hydrochrono) {
    double water_density = 1025.0;
    double wave_height = 5.0;
    double wave_period = 10.0;
    double water_depth = 40.0;
    double mean_water_level = 10.0;

    // general options
    bool visualization_on = true;
    bool statics_prestep = true;
    // solver
    auto verbose = false;
    // timestepping
    double dt = 0.1;
    // wind
    auto wind_model = std::make_shared<seahowl::env::ConstantWind>();
    wind_model->set_wind_velocity(Vector3d(0.0, 0.0, 0.0));
    wind_model->shear_coefficient = 0.12;
    // wave
    auto wave_model = std::make_shared<seahowl::env::WaveModelHydroChrono>();
    wave_model->density = water_density;
    wave_model->water_depth = water_depth;
    wave_model->mean_water_level = mean_water_level;
    auto waves_hydrochrono = std::make_shared<RegularWave>();
    waves_hydrochrono->regular_wave_amplitude_ = wave_height / 2.0;
    waves_hydrochrono->regular_wave_omega_ = 2 * seahowl::PI / wave_period;
    waves_hydrochrono->mwl_ = mean_water_level;
    waves_hydrochrono->water_depth_ = water_depth;
    waves_hydrochrono->Initialize();
    wave_model->waves = waves_hydrochrono;
    // env_model
    auto env_model = seahowl::env::EnvModel();
    env_model.add_model(wind_model);
    env_model.add_model(wave_model);

    // system
    auto system_elasto = SystemElastoChrono();
    system_elasto.set_gravitational_acceleration(Vector3d(0.0, 0.0, -9.81));
    auto system_chrono = system_elasto.chobj;

    // monopile
    auto monopile_elasto = std::make_shared<seahowl::elasto::MonopileElasto>();
    seahowl::io::populate_tower_elasto_from_file((DATADIR / "IEA15MW/monopile/monopile.csv").generic_string(),
                                                 *monopile_elasto);
    auto monopile_hydro = std::make_shared<seahowl::hydro::MonopileHydro>();
    seahowl::io::populate_tower_aero_from_file((DATADIR / "IEA15MW/monopile/monopile.csv").generic_string(),
                                               *monopile_hydro);
    monopile_hydro->discretization_fractions = {450};
    auto monopile = seahowl::core::Monopile(monopile_elasto, monopile_hydro);

    monopile.build();
    monopile_elasto->assemble(system_elasto);
    monopile.initialize(0.0, dt);

    // Setup TestFwDataSet
    TestFrameworkDataset test_dataset({false, (ref_dir / "test_monopile_hydrochrono.csv").generic_string(),
                                       (test_dir / "test_monopile_hydrochrono.test.csv").generic_string()});
    test_dataset.test_csv.add_function("time [s]", [&system_elasto]() { return system_elasto.get_time(); });
    test_dataset.test_csv.add_function("monopile base moment [Nm]",
                                       [&monopile]() { return monopile.elasto.get_tower_base_moment(); });
    test_dataset.test_csv.add_function("monopile base force [N]",
                                       [&monopile]() { return monopile.elasto.get_tower_base_force(); });
    test_dataset.test_csv.add_function("monopile top position [m]",
                                       [&monopile]() { return monopile.elasto.nodes.back()->get_position(); });

    double time = 0.0;
    while (time < 50.0) {
        // prestep
        // compute forces
        monopile_hydro->compute_env_loads(env_model, time);
        // prestep (accumulates loads from aero to elasto)
        monopile.prestep(time, dt);

        system_elasto.step(dt);
        time += dt;

        // poststep
        monopile.poststep(time, dt);
        test_dataset.test_csv.write_row();
    }

    EvaluateTest(test_dataset);
}
#endif

#ifdef HAVE_HYDRODYN
TEST_F(TestMonopile, monopile_hydrodyn) {
    // general options
    bool visualization_on = true;
    bool statics_prestep = true;
    // solver
    auto verbose = false;
    // timestepping
    double dt = 0.1;
    // wind
    auto wind_model = std::make_shared<seahowl::env::ConstantWind>();
    wind_model->set_wind_velocity(Vector3d(0.0, 0.0, 0.0));
    wind_model->shear_coefficient = 0.12;
    // wave
    auto wave_model = std::make_shared<seahowl::env::SeaStateAdapter>((root_dir / "SeaState.dat").generic_string());
    wave_model->density = 1025.0;
    // env_model
    auto env_model = seahowl::env::EnvModel();
    env_model.add_model(wind_model);
    env_model.add_model(wave_model);

    // system
    auto system_elasto = SystemElastoChrono();
    system_elasto.set_gravitational_acceleration(Vector3d(0.0, 0.0, -9.81));
    auto system_chrono = system_elasto.chobj;

    // monopile
    auto monopile_elasto = std::make_shared<seahowl::elasto::MonopileElasto>();
    seahowl::io::populate_tower_elasto_from_file((DATADIR / "IEA15MW/monopile/monopile.csv").generic_string(),
                                                 *monopile_elasto, true);

    auto monopile_hydro = std::make_shared<seahowl::hydro::MonopileHydroDyn>(
        (DATADIR / "IEA15MW/monopile/IEA-15-240-RWT-Monopile_HydroDyn.dat").generic_string());
    seahowl::io::populate_tower_aero_from_file((DATADIR / "IEA15MW/monopile/monopile.csv").generic_string(),
                                               *monopile_hydro);
    monopile_hydro->set_seastate_infile((root_dir / "SeaState.dat").generic_string());
    monopile_hydro->discretization_fractions = {45};
    auto monopile = seahowl::core::Monopile(monopile_elasto, monopile_hydro);

    monopile.build();
    monopile_elasto->assemble(system_elasto);
    monopile.initialize(0.0, dt);

    // Setup TestFwDataSet
    TestFrameworkDataset test_dataset({false, (ref_dir / "test_monopile_hydrodyn.csv").generic_string(),
                                       (test_dir / "test_monopile_hydrodyn.test.csv").generic_string()});
    test_dataset.test_csv.add_function("time [s]", [&system_elasto]() { return system_elasto.get_time(); });
    test_dataset.test_csv.add_function("monopile base moment y [Nm]",
                                       [&monopile]() { return monopile.elasto.get_tower_base_moment()[1]; });
    test_dataset.test_csv.add_function("monopile base force x [N]",
                                       [&monopile]() { return monopile.elasto.get_tower_base_force()[0]; });
    test_dataset.test_csv.add_function("monopile top position [m]",
                                       [&monopile]() { return monopile.elasto.nodes.back()->get_position(); });

    double time = 0.0;
    while (time < 50.0) {
        // prestep
        // compute forces
        monopile_hydro->compute_env_loads(env_model, time);
        // prestep (accumulates loads from aero to elasto)
        monopile.prestep(time, dt);

        system_elasto.step(dt);
        time += dt;

        // poststep
        monopile.poststep(time, dt);
        test_dataset.test_csv.write_row();
    }

    EvaluateTest(test_dataset);
}
#endif

#ifdef HAVE_SEASTATE
TEST_F(TestMonopile, monopile_seastate) {
    // general options
    bool visualization_on = true;
    bool statics_prestep = true;
    // solver
    auto verbose = false;
    // timestepping
    double dt = 0.1;
    // wind
    auto wind_model = std::make_shared<seahowl::env::ConstantWind>();
    wind_model->set_wind_velocity(Vector3d(0.0, 0.0, 0.0));
    wind_model->shear_coefficient = 0.12;
    // wave
    auto wave_model = std::make_shared<seahowl::env::SeaStateAdapter>((root_dir / "SeaState.dat").generic_string());
    wave_model->density = 1025.0;
    // env_model
    auto env_model = seahowl::env::EnvModel();
    env_model.add_model(wind_model);
    env_model.add_model(wave_model);

    // system
    auto system_elasto = SystemElastoChrono();
    system_elasto.set_gravitational_acceleration(Vector3d(0.0, 0.0, -9.81));
    auto system_chrono = system_elasto.chobj;

    // monopile
    auto monopile_elasto = std::make_shared<seahowl::elasto::MonopileElasto>();
    seahowl::io::populate_tower_elasto_from_file((DATADIR / "IEA15MW/monopile/monopile.csv").generic_string(),
                                                 *monopile_elasto);
    auto monopile_hydro = std::make_shared<seahowl::hydro::MonopileHydro>();
    seahowl::io::populate_tower_aero_from_file((DATADIR / "IEA15MW/monopile/monopile.csv").generic_string(),
                                               *monopile_hydro);
    monopile_hydro->discretization_fractions = {450};
    auto monopile = seahowl::core::Monopile(monopile_elasto, monopile_hydro);

    monopile.build();
    monopile_elasto->assemble(system_elasto);
    monopile.initialize(0.0, dt);

    // Setup TestFwDataSet
    TestFrameworkDataset test_dataset({false, (ref_dir / "test_monopile_seastate.csv").generic_string(),
                                       (test_dir / "test_monopile_seastate.test.csv").generic_string()});
    test_dataset.test_csv.add_function("time [s]", [&system_elasto]() { return system_elasto.get_time(); });
    test_dataset.test_csv.add_function("monopile base moment [Nm]",
                                       [&monopile]() { return monopile.elasto.get_tower_base_moment(); });
    test_dataset.test_csv.add_function("monopile base force [N]",
                                       [&monopile]() { return monopile.elasto.get_tower_base_force(); });
    test_dataset.test_csv.add_function("monopile top position [m]",
                                       [&monopile]() { return monopile.elasto.nodes.back()->get_position(); });

    double time = 0.0;
    while (time < 50.0) {
        // prestep
        // compute forces
        monopile_hydro->compute_env_loads(env_model, time);
        // prestep (accumulates loads from aero to elasto)
        monopile.prestep(time, dt);

        system_elasto.step(dt);
        time += dt;

        // poststep
        monopile.poststep(time, dt);
        test_dataset.test_csv.write_row();
    }

    EvaluateTest(test_dataset);
}
#endif
