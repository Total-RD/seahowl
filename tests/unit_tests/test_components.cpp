
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
#include <seahowl/env/wind_models.h>
#include <seahowl/servo/controller.h>
#include <seahowl/commons/numerics.h>
#include <seahowl/hydro/morison.h>
#include "seahowl/env/fluid_models.h"
#include "seahowl/env/wave_models.h"
#include "seahowl/env/combined_models.h"
#ifdef HAVE_HYDROCHRONO
    #include "seahowl/hydro/hydrochrono_adapter.h"
    #include "seahowl/elasto/chrono_adapters.h"
//#include <hydroc/hydro_forces.h>
#endif

#include <seahowl/io/read_json.h>

#ifdef HAVE_AERODYN
    #include <seahowl/aero/aerodyn_adapter.h>
#endif

#ifdef HAVE_INFLOWWIND
    #include "seahowl/env/inflowwind_adapter.h"
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

TEST(test_blade, mass_geometry) {
    // system
    auto system_elasto = SystemElastoChrono();
    system_elasto.set_gravitational_acceleration(Vector3d(0.0, 0.0, -9.81));

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

    // check geometry
    for (int ii = 0; ii < blade.nodes.size(); ii++) {
        ASSERT_NEAR(blade.discretized_points[ii].coordinates.x(), blade.nodes[ii]->get_position().x(), 1e-4);
        ASSERT_NEAR(blade.discretized_points[ii].coordinates.y(), blade.nodes[ii]->get_position().y(), 1e-4);
        ASSERT_NEAR(blade.discretized_points[ii].coordinates.z(), blade.nodes[ii]->get_position().z(), 1e-4);
    }

    // statics
    system_elasto.do_statics(true, 0);

    // check mass
    double blade_mass = 67051.5;
    ASSERT_NEAR(blade_mass, blade.get_mass(), 1.0);
}

TEST(test_rotor, mass) {
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
    // check mass
    double rotor_total_mass = 945690.92;
    ASSERT_NEAR(rotor_total_mass, rna.get_mass(), 1.0);
}

TEST(test_tower, mass) {
    // system
    auto system_elasto = SystemElastoChrono();

    // tower
    auto tower = seahowl::elasto::TowerElasto();
    populate_tower_elasto_from_json((DATADIR / "tower.csv").generic_string(), tower);
    tower.build();
    tower.assemble(system_elasto);

    system_elasto.do_statics(true, 0);

    // check mass
    double tower_mass = 853459.747;
    ASSERT_NEAR(tower_mass, tower.get_mass(), 1.0);
}

TEST(test_tower, frequency) {
    // system
    auto system_elasto = SystemElastoChrono();
    system_elasto.set_gravitational_acceleration(Vector3d(0.0, 0.0, -9.81));

    // tower
    auto tower = seahowl::elasto::TowerElasto();
    populate_tower_elasto_from_json((DATADIR / "tower.csv").generic_string(), tower);
    tower.discretization_fractions = {};
    tower.build();
    tower.assemble(system_elasto);
    tower.nodes.front()->set_fixed(true);

    system_elasto.do_statics(true, 0);

    // check mass
    double tower_mass = 853459.747;
    ASSERT_NEAR(tower_mass, tower.get_mass(), 1.0);

    // static position of tower top
    double pos0 = tower.nodes.back()->get_position().x();

    // check zero-crossings (static position of tower top)
    int step = 0;
    double pos_y = 0.0;
    double dt = 0.01;
    int npeaks = 0;
    double natural_period = 0.0;
    double time = 0.0;
    double end_time = 10.0;
    double start_time = 0.0;
    tower.nodes.back()->set_force(Vector3d(100000.0, 0.0, 0.0), false);
    while (time < end_time) {
        if (time > 0.5) {
            tower.nodes.back()->reset_loads();
            if (tower.nodes.back()->get_position().x() < pos0 && pos_y > pos0) {
                if (start_time == 0.0 && npeaks == 0) {
                    start_time = time;
                } else {
                    npeaks += 1;
                    natural_period = (time - start_time) / npeaks;
                }
            }
        }
        pos_y = tower.nodes.back()->get_position().x();
        system_elasto.step(dt);
        time += dt;
        step += 1;
    }

    double natural_period_ref = 1.306667;
    ASSERT_NEAR(natural_period_ref, natural_period, 0.01);
}

TEST(test_tower, cylinder_frequency) {
    // system
    auto system_elasto = SystemElastoChrono();
    system_elasto.set_gravitational_acceleration(Vector3d(0.0, 0.0, -9.81));

    // tower
    auto tower = seahowl::elasto::TowerElasto();
    // properties
    auto density = 7850.0;
    auto young_modulus = 2.11e11;
    auto poisson_ratio = 0.3;
    auto diameter = 4.0;
    auto thickness = 0.03;
    // bottom
    auto ref_point1 = seahowl::elasto::TowerReferencePointElasto();
    ref_point1.set_properties_cylinder(density, young_modulus, poisson_ratio, diameter, thickness, false);
    ref_point1.coordinates = seahowl::Vector3d(0.0, 0.0, 0.0);
    ref_point1.fraction = 0.0;
    // top
    auto ref_point2 = seahowl::elasto::TowerReferencePointElasto();
    ref_point2.set_properties_cylinder(density, young_modulus, poisson_ratio, diameter, thickness, false);
    ref_point2.coordinates = seahowl::Vector3d(0.0, 0.0, 100.0);
    ref_point2.fraction = 1.0;
    //
    tower.reference_points = {ref_point1, ref_point2};
    tower.discretization_fractions = {20};
    tower.build();
    tower.assemble(system_elasto);
    tower.nodes.front()->set_fixed(true);

    system_elasto.do_statics(true, 0);

    // check mass
    double tower_mass = 293718.5;
    ASSERT_NEAR(tower_mass, tower.get_mass(), 1.0);

    // static position of tower top
    double pos0 = tower.nodes.back()->get_position().x();

    // check zero-crossings (static position of tower top)
    int step = 0;
    double pos_y = 0.0;
    double dt = 0.01;
    int npeaks = 0;
    double natural_period = 0.0;
    double time = 0.0;
    double end_time = 10.0;
    double start_time = 0.0;
    tower.nodes.back()->set_force(Vector3d(100000.0, 0.0, 0.0), false);
    while (time < end_time) {
        if (time > 0.5) {
            tower.nodes.back()->reset_loads();
            if (tower.nodes.back()->get_position().x() < pos0 && pos_y > pos0) {
                if (start_time == 0.0 && npeaks == 0) {
                    start_time = time;
                } else {
                    npeaks += 1;
                    natural_period = (time - start_time) / npeaks;
                }
            }
        }
        pos_y = tower.nodes.back()->get_position().x();
        system_elasto.step(dt);
        time += dt;
        step += 1;
    }

    double natural_period_ref = 2.493333;  // theory: 0.4062Hz (2.4618s), f1=1.875^2/2pi*sqrt(EI/(rho*A*L^4))
    ASSERT_NEAR(natural_period_ref, natural_period, 0.01);
}

TEST(test_tower, conical_frequency) {
    // system
    auto system_elasto = SystemElastoChrono();
    system_elasto.set_gravitational_acceleration(Vector3d(0.0, 0.0, -9.81));

    // tower
    auto tower = seahowl::elasto::TowerElasto();
    // properties
    auto density = 7850.0;
    auto young_modulus = 2.11e11;
    auto poisson_ratio = 0.3;
    // bottom
    auto ref_point1 = seahowl::elasto::TowerReferencePointElasto();
    ref_point1.set_properties_cylinder(density, young_modulus, poisson_ratio, 4.0, 0.030, false);
    ref_point1.coordinates = seahowl::Vector3d(0.0, 0.0, 0.0);
    ref_point1.fraction = 0.0;
    // top
    auto ref_point2 = seahowl::elasto::TowerReferencePointElasto();
    ref_point2.set_properties_cylinder(density, young_modulus, poisson_ratio, 3.0, 0.015, false);
    ref_point2.coordinates = seahowl::Vector3d(0.0, 0.0, 100.0);
    ref_point2.fraction = 1.0;
    //
    tower.reference_points = {ref_point1, ref_point2};
    tower.discretization_fractions = {20};
    tower.build();
    tower.assemble(system_elasto);
    tower.nodes.front()->set_fixed(true);

    system_elasto.do_statics(true, 0);

    // check mass
    double tower_mass = 202070.2;
    ASSERT_NEAR(tower_mass, tower.get_mass(), 1.0);

    // static position of tower top
    double pos0 = tower.nodes.back()->get_position().x();

    // check zero-crossings (static position of tower top)
    int step = 0;
    double pos_y = 0.0;
    double dt = 0.01;
    int npeaks = 0;
    double natural_period = 0.0;
    double time = 0.0;
    double end_time = 10.0;
    double start_time = 0.0;
    tower.nodes.back()->set_force(Vector3d(100000.0, 0.0, 0.0), false);
    while (time < end_time) {
        if (time > 0.5) {
            tower.nodes.back()->reset_loads();
            if (tower.nodes.back()->get_position().x() < pos0 && pos_y > pos0) {
                if (start_time == 0.0 && npeaks == 0) {
                    start_time = time;
                } else {
                    npeaks += 1;
                    natural_period = (time - start_time) / npeaks;
                }
            }
        }
        pos_y = tower.nodes.back()->get_position().x();
        system_elasto.step(dt);
        time += dt;
        step += 1;
    }

    double natural_period_ref = 1.912500;  // theory (modal inventor): 0.515Hz (1.9417s)
    ASSERT_NEAR(natural_period_ref, natural_period, 0.01);
}

TEST(test_blade, edgewise) {
    // system
    auto system_elasto = SystemElastoChrono();
    system_elasto.set_gravitational_acceleration(Vector3d(0.0, 0.0, -9.81));

    // blade
    auto blade = seahowl::elasto::BladeElastoFEA();
    blade.fpm_mode = true;
    populate_blade_elasto_from_json((DATADIR / "blade.json").generic_string(), blade);
    // make 50 elements
    blade.discretization_fractions.clear();
    for (int ii = 0; ii < 51; ii++) {
        blade.discretization_fractions.push_back(0.02 * ii);
    }
    blade.build();
    blade.assemble(system_elasto);
    blade.nodes.front()->set_fixed(true);

    // rotate blade (flat along y axis)
    blade.rotate(PI / 2.0, Vector3d(1.0, 0.0, 0.0));
    ASSERT_NEAR(blade.reference_points.back().coordinates.y(), blade.nodes.back()->get_position().z(), 1e-4);

    // test deflection
    system_elasto.do_statics(true, 10);
    double deflection_edge = -1.093264;
    ASSERT_NEAR(deflection_edge, blade.nodes.back()->get_position().z(), 1e-4);
    // flip blade
    blade.rotate(PI, Vector3d(0.0, 1.0, 0.0));
    system_elasto.do_statics(true, 10);
    double deflection_edge2 = -1.336049;
    ASSERT_NEAR(deflection_edge2, blade.nodes.back()->get_position().z(), 1e-4);

    // static position of blade tip
    double pos0 = blade.nodes.back()->get_position().z();

    // check zero-crossings (static position of blade tip)
    int step = 0;
    double pos_y = 0.0;
    double dt = 0.02;
    int npeaks = 0;
    double natural_period = 0.0;
    double time = 0.0;
    double end_time = 10.0;
    double start_time = 0.0;
    blade.nodes.back()->set_force(Vector3d(0.0, 0.0, 1000.0), false);
    while (time < end_time) {
        if (time > 0.5) {
            blade.nodes.back()->reset_loads();
            if (blade.nodes.back()->get_position().z() < pos0 && pos_y > pos0) {
                if (start_time == 0.0 && npeaks == 0) {
                    start_time = time;
                } else {
                    npeaks += 1;
                    natural_period = (time - start_time) / npeaks;
                }
            }
        }
        pos_y = blade.nodes.back()->get_position().z();
        system_elasto.step(dt);
        time += dt;
        step += 1;
    }

    // literature edgewise natural frequency for IEA15MW: 0.642Hz (1.558s)
    double natural_period_ref = 1.436667;
    ASSERT_NEAR(natural_period_ref, natural_period, 0.01);
}

TEST(test_blade, flapwise) {
    // system
    auto system_elasto = SystemElastoChrono();
    system_elasto.set_gravitational_acceleration(Vector3d(0.0, 0.0, -9.81));

    // blade
    auto blade = seahowl::elasto::BladeElastoFEA();
    blade.fpm_mode = true;
    populate_blade_elasto_from_json((DATADIR / "blade.json").generic_string(), blade);
    // make 50 elements
    blade.discretization_fractions.clear();
    for (int ii = 0; ii < 51; ii++) {
        blade.discretization_fractions.push_back(0.02 * ii);
    }
    blade.build();
    blade.assemble(system_elasto);
    blade.nodes.front()->set_fixed(true);

    // rotate blade (flat along x axis)
    blade.rotate(PI / 2.0, Vector3d(0.0, 1.0, 0.0));
    ASSERT_NEAR(blade.reference_points.back().coordinates.x(), -blade.nodes.back()->get_position().z(), 1e-4);

    // test deflection
    system_elasto.do_statics(true, 10);
    double deflection_flap = 1.628503;
    ASSERT_NEAR(deflection_flap, blade.nodes.back()->get_position().z(), 1e-4);
    // flip blade
    blade.rotate(PI, Vector3d(1.0, 0.0, 0.0));
    system_elasto.do_statics(true, 10);
    double deflection_flap2 = -6.059450;
    ASSERT_NEAR(deflection_flap2, blade.nodes.back()->get_position().z(), 1e-4);

    // static position of blade tip
    double pos0 = blade.nodes.back()->get_position().z();

    // check zero-crossings (static position of blade tip)
    int step = 0;
    double pos_y = 0.0;
    double dt = 0.02;
    int npeaks = 0;
    double natural_period = 0.0;
    double time = 0.0;
    double end_time = 10.0;
    double start_time = 0.0;
    blade.nodes.back()->set_force(Vector3d(0.0, 0.0, 1000.0), false);
    while (time < end_time) {
        if (time > 0.5) {
            blade.nodes.back()->reset_loads();
            if (blade.nodes.back()->get_position().z() < pos0 && pos_y > pos0) {
                if (start_time == 0.0 && npeaks == 0) {
                    start_time = time;
                } else {
                    npeaks += 1;
                    natural_period = (time - start_time) / npeaks;
                }
            }
        }
        pos_y = blade.nodes.back()->get_position().z();
        system_elasto.step(dt);
        time += dt;
        step += 1;
    }

    // literature flapwise natural frequency for IEA15MW: 0.555Hz (1.802s)
    double natural_period_ref = 1.980000;
    ASSERT_NEAR(natural_period_ref, natural_period, 0.01);
}

TEST(test_bemt, tower_shadow_check) {
    // system
    auto system_elasto = SystemElastoChrono();
    system_elasto.set_gravitational_acceleration(Vector3d(0.0, 0.0, -9.81));

    // tower
    auto tower_elasto = seahowl::elasto::TowerElasto();
    auto tower_aero = seahowl::aero::TowerAero();
    auto tower = seahowl::core::Tower(tower_elasto, tower_aero);
    populate_tower_from_json((DATADIR / "tower.csv").generic_string(), tower);

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
    ASSERT_NEAR(wind_velocity1.x(), 9.100318, 1e-4);
    ASSERT_NEAR(wind_velocity1.y(), 2.625343, 1e-4);
    ASSERT_NEAR(wind_velocity1.z(), 1.0, 1e-4);

    // position 2
    auto position2 = Vector3d(-15.0, 0.0, 45.0);
    Vector3d wind_velocity2 = wind_velocity;
    seahowl::aero::apply_tower_shadow_effect_on_wind(wind_velocity2, position2, tower_aero);
    ASSERT_NEAR(wind_velocity2.x(), 9.047282, 1e-4);
    ASSERT_NEAR(wind_velocity2.y(), 3.285815, 1e-4);
    ASSERT_NEAR(wind_velocity2.z(), 1.0, 1e-4);

    // position 3
    auto position3 = Vector3d(-16.0, 2.0, 20.0);
    Vector3d wind_velocity3 = wind_velocity;
    seahowl::aero::apply_tower_shadow_effect_on_wind(wind_velocity3, position3, tower_aero);
    ASSERT_NEAR(wind_velocity3.x(), 9.227646, 1e-4);
    ASSERT_NEAR(wind_velocity3.y(), 3.463146, 1e-4);
    ASSERT_NEAR(wind_velocity3.z(), 1.0, 1e-4);

    auto wind_velocity_xz = Vector3d(10.0, 0.0, 1.0);

    // position 4
    auto position4 = Vector3d(-6.0, 2.0, 20.0);
    Vector3d wind_velocity4 = wind_velocity_xz;
    seahowl::aero::apply_tower_shadow_effect_on_wind(wind_velocity4, position4, tower_aero);
    ASSERT_NEAR(wind_velocity4.x(), 5.514508, 1e-4);
    ASSERT_NEAR(wind_velocity4.y(), 3.364119, 1e-4);
    ASSERT_NEAR(wind_velocity4.z(), 1.0, 1e-4);

    // position 5
    auto position5 = Vector3d(-16.0, -2.0, 20.0);
    Vector3d wind_velocity5 = wind_velocity_xz;
    seahowl::aero::apply_tower_shadow_effect_on_wind(wind_velocity5, position5, tower_aero);
    ASSERT_NEAR(wind_velocity5.x(), 9.163947, 1e-4);
    ASSERT_NEAR(wind_velocity5.y(), -0.212330, 1e-4);
    ASSERT_NEAR(wind_velocity5.z(), 1.0, 1e-4);

    // position 5 wind rotation
    auto rot2 = AngleAxisd(PI / 36.0, Vector3d(0.0, 0.0, 1.0));
    auto position5_rot2 = rot2 * position5;
    Vector3d wind_velocity5_rot2 = rot2 * wind_velocity_xz;
    seahowl::aero::apply_tower_shadow_effect_on_wind(wind_velocity5_rot2, position5_rot2, tower_aero);
    auto rot2_wind = rot2.inverse() * wind_velocity5_rot2;
    ASSERT_NEAR(rot2_wind.x(), 9.163947, 1e-4);
    ASSERT_NEAR(rot2_wind.y(), -0.212330, 1e-4);
    ASSERT_NEAR(rot2_wind.z(), 1.0, 1e-4);

    // position 5 tower and wind rotation
    auto rot = AngleAxisd(PI / 36.0, Vector3d(0.0, 1.0, 0.0));
    auto position5_rot = rot * position5;
    for (auto& node : tower_aero.nodes) {
        node.set_position(rot * node.get_position());
    }
    Vector3d wind_velocity5_rot = rot * wind_velocity_xz;
    seahowl::aero::apply_tower_shadow_effect_on_wind(wind_velocity5_rot, position5_rot, tower_aero);
    auto rot_wind = rot.inverse() * wind_velocity5_rot;
    ASSERT_NEAR(rot_wind.x(), 9.163947, 1e-4);
    ASSERT_NEAR(rot_wind.y(), -0.212330, 1e-4);
    ASSERT_NEAR(rot_wind.z(), 1.0, 1e-4);
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
    auto wind_model = seahowl::env::ConstantWind();
    wind_model.set_wind_velocity(Vector3d(8.0, 0.0, 0.0));
    wind_model.shear_coefficient = 0.12;
    // turbine
    double initial_pitch = seahowl::PI / 8.0;

    // system
    auto system_elasto = SystemElastoChrono();
    system_elasto.set_gravitational_acceleration(Vector3d(0.0, 0.0, -9.81));
    auto system_chrono = system_elasto.chobj;

    // turbine
    auto turbine_elasto = seahowl::elasto::TurbineElasto();
    auto turbine_aero = seahowl::aero::TurbineAero();
    auto turbine = seahowl::core::Turbine(turbine_elasto, turbine_aero);
    populate_turbine_from_json((DATADIR / "turbine_nocontrol.json").generic_string(), turbine);

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
    }

    ASSERT_NEAR(turbine.rna.elasto.get_rpm(), 2.687975, 1e-4);
}

TEST(test_turbine, rpm_initial_pitch_fpm) {
    // general options
    bool visualization_on = true;
    bool statics_prestep = true;
    // solver
    auto verbose = false;
    // timestepping
    double dt = 0.1;
    // wind
    auto wind_model = seahowl::env::ConstantWind();
    wind_model.set_wind_velocity(Vector3d(8.0, 0.0, 0.0));
    wind_model.shear_coefficient = 0.12;
    // turbine
    double initial_pitch = seahowl::PI / 8.0;

    // system
    auto system_elasto = SystemElastoChrono();
    system_elasto.set_gravitational_acceleration(Vector3d(0.0, 0.0, -9.81));
    auto system_chrono = system_elasto.chobj;

    // turbine
    auto turbine_elasto = seahowl::elasto::TurbineElasto();
    auto turbine_aero = seahowl::aero::TurbineAero();
    auto turbine = seahowl::core::Turbine(turbine_elasto, turbine_aero);
    populate_turbine_from_json((DATADIR / "turbine_nocontrol_fpm.json").generic_string(), turbine);

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
    }

    ASSERT_NEAR(turbine.rna.elasto.get_rpm(), 2.691299, 1e-4);
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
    auto wind_model = seahowl::env::ConstantWind();
    wind_model.set_wind_velocity(Vector3d(8.0, 0.0, 0.0));
    wind_model.shear_coefficient = 0.12;
    // turbine
    double initial_pitch = seahowl::PI / 8.0;

    // system
    auto system_elasto = SystemElastoChrono();
    system_elasto.set_gravitational_acceleration(Vector3d(0.0, 0.0, -9.81));
    auto system_chrono = system_elasto.chobj;

    // turbine
    auto turbine_elasto = seahowl::elasto::TurbineElasto();
    auto turbine_aero = seahowl::aero::TurbineAero();
    auto turbine = seahowl::core::Turbine(turbine_elasto, turbine_aero);
    populate_turbine_from_json((DATADIR / "turbine_nocontrol_rigid.json").generic_string(), turbine);

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
    }

    ASSERT_NEAR(turbine.rna.elasto.get_rpm(), 2.709388, 1e-4);
}

TEST(test_turbine, controller_target_rpm) {
    // general options
    bool visualization_on = true;
    bool statics_prestep = true;
    // solver
    auto verbose = false;
    // timestepping
    double dt = 0.05;
    // wind
    auto wind_model = seahowl::env::ConstantWind();
    wind_model.set_wind_velocity(Vector3d(8.0, 0.0, 0.0));
    wind_model.shear_coefficient = 0.12;
    // turbine
    double initial_pitch = seahowl::PI / 8.0;

    // system
    auto system_elasto = SystemElastoChrono();
    system_elasto.set_gravitational_acceleration(Vector3d(0.0, 0.0, -9.81));
    auto system_chrono = system_elasto.chobj;

    // turbine
    auto turbine_elasto = seahowl::elasto::TurbineElasto();
    auto turbine_aero = seahowl::aero::TurbineAero();
    auto turbine = seahowl::core::Turbine(turbine_elasto, turbine_aero);
    populate_turbine_from_json((DATADIR / "turbine_nocontrol_rigid.json").generic_string(), turbine);

    // remove controller
    double target_rpm = 2.0;
    auto controller = std::make_shared<seahowl::servo::ControllerVariableTorque>();
    controller->target_rpm = target_rpm;
    turbine.controller = controller;
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
    while (time < 100) {
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
    }

    ASSERT_NEAR(turbine.rna.elasto.get_rpm(), target_rpm, 1e-2);
}

TEST(test_turbine, actuator_disk) {
    // general options
    bool visualization_on = true;
    bool statics_prestep = true;
    // solver
    auto verbose = false;
    // timestepping
    double dt = 0.1;
    // wind
    auto wind_model = seahowl::env::ConstantWind();
    wind_model.set_wind_velocity(Vector3d(11.0, 0.0, 0.0));
    wind_model.shear_coefficient = 0.12;
    // turbine
    double initial_pitch = 0.0 * seahowl::PI / 1000.0;
    // power target
    auto reference_power = 15.5e6;

    // system
    auto system_elasto = SystemElastoChrono();
    system_elasto.set_gravitational_acceleration(Vector3d(0.0, 0.0, -9.81));
    auto system_chrono = system_elasto.chobj;

    // turbine
    auto turbine_elasto = seahowl::elasto::TurbineElasto();
    auto turbine_aero = seahowl::aero::TurbineAero();
    auto turbine = seahowl::core::Turbine(turbine_elasto, turbine_aero);
    populate_turbine_from_json((DATADIR / "turbine_disk.json").generic_string(), turbine);

    // remove controller
    double target_rpm = 7.56;
    auto controller = std::make_shared<seahowl::servo::ControllerVariableTorque>();
    controller->target_rpm = target_rpm;
    turbine.controller = controller;
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
    while (time < 100) {
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
    }

    // ASSERT_NEAR(turbine.rna.elasto.get_rpm(), target_rpm, 1e-3);
    ASSERT_NEAR(turbine.get_generated_power(), reference_power, reference_power * 0.01);
    ASSERT_NEAR(turbine.rna.elasto.get_rpm(), target_rpm, target_rpm * 0.01);

    reference_power = 15.3e6;
    // wind
    wind_model.set_wind_velocity(Vector3d(15.0, 0.0, 0.0));
    // turbine
    initial_pitch = 11.0 * seahowl::PI / 180.0;
    turbine.rna.elasto.rotor->apply_collective_pitch_increment(initial_pitch);
    while (time < 200) {
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
    }

    ASSERT_NEAR(turbine.get_generated_power(), reference_power, reference_power * 0.01);
    ASSERT_NEAR(turbine.rna.elasto.get_rpm(), target_rpm, target_rpm * 0.01);
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
    populate_turbine_from_json((DATADIR / "turbine_nocontrol_aerodyn.json").generic_string(), turbine);
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
    }

    ASSERT_NEAR(turbine.rna.elasto.get_rpm(), 2.719950, 1e-4);
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
    auto wind_model = std::make_shared<seahowl::env::ConstantWind>();
    wind_model->set_wind_velocity(Vector3d(8.0, 0.0, 0.0));
    wind_model->shear_coefficient = 0.12;
    // turbine
    double initial_pitch = seahowl::PI / 8.0;

    // system
    auto system_elasto = SystemElastoChrono();
    system_elasto.set_gravitational_acceleration(Vector3d(0.0, 0.0, -9.81));
    auto system_aero = seahowl::aero::SystemAero();

    // system core
    auto system_core = seahowl::core::System(system_elasto, system_aero);
    system_core.fluid_model = wind_model;

    // turbines
    auto turbine_file = (DATADIR / "turbine_nocontrol_rigid.json").generic_string();
    auto nturbines = 3;
    for (int ii = 0; ii < nturbines; ii++) {
        system_core.elasto.turbines.push_back(std::make_shared<seahowl::elasto::TurbineElasto>());
        system_core.aero.turbines.push_back(std::make_shared<seahowl::aero::TurbineAero>());
        system_core.turbines.push_back(std::make_shared<seahowl::core::Turbine>(*system_core.elasto.turbines.back(),
                                                                                *system_core.aero.turbines.back()));
        auto& turbine = *system_core.turbines.back();
        populate_turbine_from_json(turbine_file, turbine);
        // empty controller
        turbine.controller = std::make_shared<seahowl::servo::Controller>();
        // translate
        turbine.build();
        turbine.elasto.translate(Vector3d(0.0 + ii * 150.0, 0.0 + ii * (-150.0), 0.0));
        // fix
        turbine.tower.elasto.nodes.front()->set_fixed(true);
    }

    // assemble system
    system_core.elasto.assemble();

    // statics
    if (statics_prestep) {
        system_elasto.do_statics(true, 10);
    }

    double time = 0.0;
    for (auto& turbine : system_core.turbines) {
        turbine->rna.elasto.rotor->apply_collective_pitch_increment(initial_pitch);
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
        ASSERT_NEAR(turbine->rna.elasto.get_rpm(), 2.728150, 0.02);
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
    }

    ASSERT_NEAR(turbine.rna.elasto.get_rpm(), 2.687975, 1e-4);
}
#endif

TEST(Morison_test, MCF_Table_test) {
    seahowl::hydro::MacCamyFuchsTable mytable = seahowl::hydro::MacCamyFuchsTable();
    mytable.wave_peak_period = 10.0;

    // general options
    seahowl::hydro::HydroCoefficients coefficients;
    auto node1 = seahowl::hydro::MorisonNode();
    coefficients.use_MacCamyFuchs_correction = true;

    node1.coefficients = coefficients;

    double lambda = 1.56 * mytable.wave_peak_period * mytable.wave_peak_period;

    double DNV_table_X0 = 0.0018;
    double DNV_table_Y0 = 2.003;
    node1.diameter = 0.0018 * lambda;
    double Table_Y0 = mytable.interpolateCmBinarySearch(node1.diameter);
    ASSERT_NEAR(DNV_table_Y0, Table_Y0, 0.01);

    double DNV_table_X2 = 0.128;
    double DNV_table_Y2 = 2.060;
    node1.diameter = 0.128 * lambda;
    double Table_Y2 = mytable.interpolateCmBinarySearch(node1.diameter);
    ASSERT_NEAR(DNV_table_Y2, Table_Y2, 0.01);

    double DNV_table_X4 = 0.322;
    double DNV_table_Y4 = 1.360;
    node1.diameter = 0.322 * lambda;
    double Table_Y4 = mytable.interpolateCmBinarySearch(node1.diameter);
    ASSERT_NEAR(DNV_table_Y4, Table_Y4, 0.01);

    double DNV_table_X6 = 0.576;
    double DNV_table_Y6 = 0.655;
    node1.diameter = 0.576 * lambda;
    double Table_Y6 = mytable.interpolateCmBinarySearch(node1.diameter);
    ASSERT_NEAR(DNV_table_Y6, Table_Y6, 0.01);

    double DNV_table_X8 = 0.864;
    double DNV_table_Y8 = 0.360;
    node1.diameter = 0.864 * lambda;
    double Table_Y8 = mytable.interpolateCmBinarySearch(node1.diameter);
    ASSERT_NEAR(DNV_table_Y8, Table_Y8, 0.01);
}

TEST(Morison_test, Cd_Table) {
    seahowl::hydro::MacCamyFuchsTable mytable = seahowl::hydro::MacCamyFuchsTable();
    mytable.wave_peak_period = 10.0;

    // general options
    seahowl::hydro::HydroCoefficients coefficients;
    auto node1 = seahowl::hydro::MorisonNode();
    coefficients.use_Cd_correction = true;

    node1.coefficients = coefficients;

    double fluid_velocity = 0.08431;
    node1.diameter = 8.1;
    double CD1 = mytable.getCd(node1.diameter, mytable.wave_peak_period,
                               fluid_velocity);  // double diameter, double wave_period, double fluid_velocity

    double CD_ref = 1.045;
    ASSERT_NEAR(CD_ref, CD1, 0.01);
}
