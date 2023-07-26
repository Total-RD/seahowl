
#include <gtest/gtest.h>
#include <cmath>
#include <memory>

#include <seahowl/elasto/blade_elasto.h>
#include <seahowl/elasto/rotor_elasto.h>
#include <seahowl/elasto/tower_elasto.h>
#include <seahowl/elasto/chrono_adapters.h>
#include <seahowl/core/blade.h>
#include <seahowl/core/rotor.h>
#include <seahowl/core/turbine.h>
#include <seahowl/core/system.h>
#include <seahowl/aero/bemt.h>
#include <seahowl/aero/system_aero.h>
#include <seahowl/aero/wind_models.h>
#include <seahowl/servo/controller.h>
#include <seahowl/commons/numerics.h>

#include <seahowl/io/read_json.h>

#ifdef HAVE_AERODYN
    #include <seahowl/aero/aerodyn_adapter.h>
#endif

#ifdef HAVE_INFLOWWIND
    #include "seahowl/aero/inflowwind_adapter.h"
#endif

using namespace seahowl::elasto;
using namespace seahowl;

#include <filesystem>  // C++17
#include <cstdlib>

using std::filesystem::path;
using std::filesystem::absolute;

static path DATADIR{};

int main(int argc, char** argv) {
    const char* env_p = std::getenv("SEAHOWL_DATADIR");

    if (env_p == nullptr) {
        if (argc < 2) {
            std::cerr << "Usage: test_01.exe [<datadir>] or set SEAHOWL_DATADIR environment variable" << std::endl;
            return 1;
        } else {
            DATADIR = absolute(path(argv[1]));
        }
    } else {
        DATADIR = absolute(path(env_p));
    }

    ::testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}

TEST(test_blade, mass_deflection) {
    // system
    auto system_elasto = SystemElastoChrono();
    system_elasto.set_gravitational_acceleration(Vector3d(0.0, -9.81, 0.0));

    // blade
    auto blade = seahowl::elasto::BladeElastoFEA();
    blade.reference_points = get_blade_elasto_reference_points_from_json((DATADIR / "blade.json").generic_string());
    // make 50 elements
    blade.discretization_fractions.clear();
    for (int ii = 0; ii < 51; ii++) {
        blade.discretization_fractions.push_back(0.02 * ii);
    }
    blade.build();
    blade.assemble(system_elasto);
    blade.nodes.front()->set_fixed(true);

    system_elasto.do_statics(true, 0);

    // check mass
    double blade_mass = 67051.5;
    ASSERT_NEAR(blade_mass, blade.get_mass(), 1.0);

    // check deflection from gravity (edge)
    double deflection_edge = -0.8580;
    blade.rotate(PI, Vector3d(0.0, 0.0, 1.0));
    system_elasto.do_statics(true, 0);
    ASSERT_NEAR(deflection_edge, blade.nodes.back()->get_position().y(), 0.001);

    // check deflection from gravity (flap)
    double deflection_flap = 2.91411;
    blade.rotate(PI / 2.0, Vector3d(0.0, 0.0, 1.0));
    system_elasto.do_statics(true, 0);
    ASSERT_NEAR(deflection_flap, blade.nodes.back()->get_position().y(), 0.001);
}

TEST(test_rotor, mass) {
    // system
    auto system_elasto = SystemElastoChrono();
    system_elasto.set_gravitational_acceleration(Vector3d(0.0, -9.81, 0.0));

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
    populate_rotor_elasto_from_json((DATADIR / "rna.json").generic_string(), rna);
    rna.rotor->blades = blades;
    rna.build();
    rna.assemble(system_elasto);
    rna.body_yaw_bearing->set_fixed(true);

    system_elasto.do_statics(true, 0);
    // check mass
    double rotor_total_mass = 945690.92;
    ASSERT_NEAR(rotor_total_mass, rna.get_mass(), 1.0);
}

TEST(test_tower, mass) {
    // system
    auto system_elasto = SystemElastoChrono();
    system_elasto.set_gravitational_acceleration(Vector3d(0.0, -9.81, 0.0));

    // tower
    auto tower = seahowl::elasto::TowerElasto();
    populate_tower_elasto_from_json((DATADIR / "tower.json").generic_string(), tower);
    tower.build();
    tower.assemble(system_elasto);

    system_elasto.do_statics(true, 0);

    // check mass
    double tower_mass = 870391.59776;
    ASSERT_NEAR(tower_mass, tower.get_mass(), 1.0);
}

TEST(test_blade, natural_period_dynamic_edge) {
    // system
    auto system_elasto = SystemElastoChrono();
    system_elasto.set_gravitational_acceleration(Vector3d(0.0, -9.81, 0.0));

    // blade
    auto blade = seahowl::elasto::BladeElastoFEA();
    populate_blade_elasto_from_json((DATADIR / "blade.json").generic_string(), blade);
    // make 50 elements
    blade.discretization_fractions.clear();
    for (int ii = 0; ii < 51; ii++) {
        blade.discretization_fractions.push_back(0.02 * ii);
    }
    blade.build();
    blade.assemble(system_elasto);
    blade.nodes.front()->set_fixed(true);

    system_elasto.do_statics(true, 0);

    // static position of blade tip
    double pos0 = blade.nodes.back()->get_position().y();

    // check zero-crossings (static position of blade tip)
    int step = 0;
    double pos_y = 0.0;
    double dt = 0.02;
    int npeaks = 0;
    double natural_period = 0.0;
    double time = 0.0;
    double end_time = 10.0;
    double start_time = 0.0;
    blade.nodes.back()->set_force(Vector3d(0.0, 1000.0, 0.0), false);
    while (time < end_time) {
        if (time > 0.5) {
            blade.nodes.back()->reset_loads();
            if (blade.nodes.back()->get_position().y() < pos0 && pos_y > pos0) {
                if (start_time == 0.0) {
                    start_time = time;
                } else {
                    npeaks += 1;
                    natural_period = (time - start_time) / npeaks;
                }
            }
        }
        pos_y = blade.nodes.back()->get_position().y();
        system_elasto.step(dt);
        time += dt;
        step += 1;
    }

    // literature edgewise natural frequency for IEA15MW: 0.642Hz (1.558s)
    double natural_period_ref = 1.343;
    ASSERT_NEAR(natural_period_ref, natural_period, 0.01);
}

TEST(test_blade, natural_period_dynamic_flap) {
    // system
    auto system_elasto = SystemElastoChrono();
    system_elasto.set_gravitational_acceleration(Vector3d(0.0, -9.81, 0.0));

    // blade
    auto blade = seahowl::elasto::BladeElastoFEA();
    populate_blade_elasto_from_json((DATADIR / "blade.json").generic_string(), blade);
    // make 50 elements
    blade.discretization_fractions.clear();
    for (int ii = 0; ii < 51; ii++) {
        blade.discretization_fractions.push_back(0.02 * ii);
    }
    blade.build();
    blade.assemble(system_elasto);
    blade.nodes.front()->set_fixed(true);

    // rotate blade for flap
    blade.rotate(-PI / 2.0, Vector3d(0.0, 0.0, 1.0));

    system_elasto.do_statics(true, 0);

    // static position of blade tip
    double pos0 = blade.nodes.back()->get_position().y();

    // check zero-crossings (static position of blade tip)
    int step = 0;
    double pos_y = 0.0;
    double dt = 0.02;
    int npeaks = 0;
    double natural_period = 0.0;
    double time = 0.0;
    double end_time = 10.0;
    double start_time = 0.0;
    blade.nodes.back()->set_force(Vector3d(0.0, 1000.0, 0.0), false);
    while (time < end_time) {
        if (time > 0.5) {
            blade.nodes.back()->reset_loads();
            if (blade.nodes.back()->get_position().y() < pos0 && pos_y > pos0) {
                if (start_time == 0.0) {
                    start_time = time;
                } else {
                    npeaks += 1;
                    natural_period = (time - start_time) / npeaks;
                }
            }
        }
        pos_y = blade.nodes.back()->get_position().y();
        system_elasto.step(dt);
        time += dt;
        step += 1;
    }

    // literature flapwise natural frequency for IEA15MW: 0.555Hz (1.802s)
    double natural_period_ref = 1.92;
    ASSERT_NEAR(natural_period_ref, natural_period, 0.01);
}

TEST(test_tower, tower_shadow_check) {
    // system
    auto system_elasto = SystemElastoChrono();

    // tower
    auto tower_elasto = seahowl::elasto::TowerElasto();
    auto tower_aero = seahowl::aero::TowerAero();
    auto tower = seahowl::core::Tower(tower_elasto, tower_aero);
    populate_tower_from_json((DATADIR / "tower.json").generic_string(), tower);

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
    ASSERT_NEAR(wind_velocity1.x(), 9.223643, 0.001);

    // position 2
    auto position2 = Vector3d(-15.0, 0.0, 45.0);
    Vector3d wind_velocity2 = wind_velocity;
    seahowl::aero::apply_tower_shadow_effect_on_wind(wind_velocity2, position2, tower_aero);
    ASSERT_NEAR(wind_velocity2.x(), 8.964047, 0.001);

    // position 3
    auto position3 = Vector3d(-16.0, 2.0, 20.0);
    Vector3d wind_velocity3 = wind_velocity;
    seahowl::aero::apply_tower_shadow_effect_on_wind(wind_velocity3, position3, tower_aero);
    ASSERT_NEAR(wind_velocity3.x(), 9.056021, 0.001);
}

TEST(test_turbine, rpm_initial_pitch) {
    // general options
    bool visualization_on = true;
    bool statics_prestep = true;
    // solver
    auto verbose = false;
    // timestepping
    double dt = 0.1;
    // wind
    auto wind_model = seahowl::aero::ConstantWind();
    wind_model.set_wind_velocity(Vector3d(8.0, 0.0, 0.0));
    // turbine
    double initial_pitch = seahowl::PI / 8.0;

    // system
    auto system_elasto = SystemElastoChrono();
    system_elasto.set_gravitational_acceleration(Vector3d(0.0, -9.81, 0.0));
    auto system_chrono = system_elasto.chobj;

    // turbine
    auto turbine_elasto = seahowl::elasto::TurbineElasto();
    auto turbine_aero = seahowl::aero::TurbineAero();
    auto turbine = seahowl::core::Turbine(turbine_elasto, turbine_aero);
    populate_turbine_from_json((DATADIR / "turbine_nocontrol.json").generic_string(), turbine);

    turbine.aero.use_aerodyn = false;

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
    // while (application.GetDevice()->run()) {
    while (time < 50) {
        // prestep
        // compute forces
        turbine.aero.compute_aero_loads(wind_model, time);
        // prestep (accumulates loads from aero to elasto)
        turbine.prestep(time, dt);

        system_elasto.step(dt);
        time += dt;

        // poststep
        turbine.poststep(time, dt);
    }

    ASSERT_NEAR(turbine.rna.elasto.get_rpm(), 2.828219, 1e-4);
}

TEST(test_turbine, rpm_initial_pitch_rigid_rotor) {
    // general options
    bool visualization_on = true;
    bool statics_prestep = true;
    // solver
    auto verbose = false;
    // timestepping
    double dt = 0.1;
    // wind
    auto wind_model = seahowl::aero::ConstantWind();
    wind_model.set_wind_velocity(Vector3d(8.0, 0.0, 0.0));
    // turbine
    double initial_pitch = seahowl::PI / 8.0;

    // system
    auto system_elasto = SystemElastoChrono();
    system_elasto.set_gravitational_acceleration(Vector3d(0.0, -9.81, 0.0));
    auto system_chrono = system_elasto.chobj;

    // turbine
    auto turbine_elasto = seahowl::elasto::TurbineElasto();
    auto turbine_aero = seahowl::aero::TurbineAero();
    auto turbine = seahowl::core::Turbine(turbine_elasto, turbine_aero);
    populate_turbine_from_json((DATADIR / "turbine_nocontrol_rigid.json").generic_string(), turbine);

    turbine.aero.use_aerodyn = false;

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
    // while (application.GetDevice()->run()) {
    while (time < 50) {
        // prestep
        // compute forces
        turbine.aero.compute_aero_loads(wind_model, time);
        // prestep (accumulates loads from aero to elasto)
        turbine.prestep(time, dt);

        system_elasto.step(dt);
        time += dt;

        // poststep
        turbine.poststep(time, dt);
    }

    ASSERT_NEAR(turbine.rna.elasto.get_rpm(), 2.814399, 1e-4);
}

#ifdef HAVE_AERODYN
TEST(test_aerodyn, rpm_initial_pitch) {
    // general options
    bool visualization_on = true;
    bool statics_prestep = true;
    // solver
    auto verbose = false;
    // timestepping
    double dt = 0.1;
    // wind
    auto wind_model = seahowl::aero::ConstantWind();
    wind_model.set_wind_velocity(Vector3d(8.0, 0.0, 0.0));
    // turbine
    double initial_pitch = seahowl::PI / 8.0;

    // system
    auto system_elasto = SystemElastoChrono();
    system_elasto.set_gravitational_acceleration(Vector3d(0.0, -9.81, 0.0));

    // turbine
    auto turbine_elasto = seahowl::elasto::TurbineElasto();
    auto turbine_aero = seahowl::aero::TurbineAero();
    auto turbine = seahowl::core::Turbine(turbine_elasto, turbine_aero);
    populate_turbine_from_json((DATADIR / "turbine_nocontrol.json").generic_string(), turbine);
    // remove controller
    turbine.controller = std::make_shared<seahowl::servo::Controller>();

    turbine.aero.use_aerodyn = true;

    turbine.aero.aerodyn = std::make_shared<seahowl::aero::AeroDynAdapter>(
        (DATADIR / "aerodyn/IEA-15-240-RWT_AeroDyn15.dat").generic_string(),
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
    turbine.rna.elasto.apply_collective_pitch_increment(initial_pitch);
    turbine.initialize(time, dt);
    // while (application.GetDevice()->run()) {
    while (time < 50) {
        // prestep
        // compute forces
        turbine.aero.compute_aero_loads(wind_model, time);
        // prestep (accumulates loads from aero to elasto)
        turbine.prestep(time, dt);

        system_elasto.step(dt);
        time += dt;

        // poststep
        turbine.poststep(time, dt);
    }

    ASSERT_NEAR(turbine.rna.elasto.get_rpm(), 2.77, 0.02);
}
#endif

TEST(test_turbine, multiturbines) {
    // general options
    bool visualization_on = true;
    bool statics_prestep = true;
    // solver
    auto verbose = false;
    // timestepping
    double dt = 0.1;
    // wind
    auto wind_model = std::make_shared<seahowl::aero::ConstantWind>();
    wind_model->set_wind_velocity(Vector3d(8.0, 0.0, 0.0));
    // turbine
    double initial_pitch = seahowl::PI / 8.0;

    // system
    auto system_elasto = std::make_shared<SystemElastoChrono>();
    system_elasto->set_gravitational_acceleration(Vector3d(0.0, -9.81, 0.0));

    // system core
    seahowl::core::System system_core;
    system_core.wind_model = wind_model;
    system_core.system_elasto = system_elasto;
    system_core.system_aero = std::make_shared<seahowl::aero::SystemAero>();

    // turbines
    auto turbine_file = (DATADIR / "turbine_nocontrol_rigid.json").generic_string();
    auto nturbines = 3;
    for (int ii = 0; ii < nturbines; ii++) {
        system_core.system_elasto->turbines.push_back(seahowl::elasto::TurbineElasto());
        system_core.system_aero->turbines.push_back(seahowl::aero::TurbineAero());
        system_core.turbines.push_back(seahowl::core::Turbine(system_core.system_elasto->turbines.back(),
                                                              system_core.system_aero->turbines.back()));
        auto& turbine = system_core.turbines.back();
        populate_turbine_from_json(turbine_file, turbine);
        // empty controller
        turbine.controller = std::make_shared<seahowl::servo::Controller>();
        // translate
        turbine.build();
        turbine.translate(Vector3d(0.0 + ii * 150.0, 0.0 + ii * (-150.0), 0.0));
        // fix
        turbine.tower.elasto.nodes.front()->set_fixed(true);
    }

    // assemble system
    system_core.assemble();

    // statics
    if (statics_prestep) {
        system_elasto->do_statics(true, 10);
    }

    double time = 0.0;
    for (auto& turbine : system_core.turbines) {
        turbine.rna.elasto.rotor->apply_collective_pitch_increment(initial_pitch);
    }
    system_core.initialize(time, dt);
    while (time < 50) {
        // prestep
        system_core.prestep(time, dt);

        // step
        system_core.step(dt);
        time += dt;

        // poststep
        system_core.poststep(time, dt);
    }

    for (auto& turbine : system_core.turbines) {
        ASSERT_NEAR(turbine.rna.elasto.get_rpm(), 2.829, 0.02);
    }
}

#ifdef HAVE_INFLOWWIND
TEST(test_inflowwind, rpm_initial_pitch) {
    // general options
    bool visualization_on = true;
    bool statics_prestep = true;
    // solver
    auto verbose = false;
    // timestepping
    double dt = 0.1;
    // wind
    auto wind_model =
        seahowl::aero::InflowWindAdapter((DATADIR / "aerodyn/IEA-15-240-RWT_InflowWind.dat").generic_string(),
                                         (DATADIR / "aerodyn/long_step_wind.wnd").generic_string());
    wind_model.init(dt);
    // turbine
    double initial_pitch = seahowl::PI / 8.0;

    // system
    auto system_elasto = SystemElastoChrono();
    system_elasto.set_gravitational_acceleration(Vector3d(0.0, -9.81, 0.0));

    // turbine
    auto turbine_elasto = seahowl::elasto::TurbineElasto();
    auto turbine_aero = seahowl::aero::TurbineAero();
    auto turbine = seahowl::core::Turbine(turbine_elasto, turbine_aero);
    populate_turbine_from_json((DATADIR / "turbine_nocontrol.json").generic_string(), turbine);
    // remove controller
    turbine.controller = std::make_shared<seahowl::servo::Controller>();

    turbine.aero.use_aerodyn = false;

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
    turbine.rna.elasto.apply_collective_pitch_increment(initial_pitch);
    turbine.initialize(time, dt);
    // while (application.GetDevice()->run()) {
    while (time < 50) {
        // prestep
        // compute forces
        turbine.aero.compute_aero_loads(wind_model, time);
        // prestep (accumulates loads from aero to elasto)
        turbine.prestep(time, dt);

        system_elasto.step(dt);
        time += dt;

        // poststep
        turbine.poststep(time, dt);
    }

    ASSERT_NEAR(turbine.rna.elasto.get_rpm(), 2.829, 0.02);
}
#endif
