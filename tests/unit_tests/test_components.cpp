
#include <gtest/gtest.h>
#include <cmath>

#include <chrono/physics/ChSystemSMC.h>
#include <chrono/solver/ChDirectSolverLS.h>
#include <chrono/solver/ChIterativeSolverLS.h>

#include <seahowl/elasto/blade_elasto.h>
#include <seahowl/elasto/rotor_elasto.h>
#include <seahowl/elasto/tower_elasto.h>
#include <seahowl/core/utils.h>
#include <seahowl/core/blade.h>
#include <seahowl/core/rotor.h>
#include <seahowl/elasto/tower_elasto.h>
#include <seahowl/core/turbine.h>
#include <seahowl/core/system.h>

#include <seahowl/io/read_json.h>

#ifdef HAVE_AERODYN
    #include <seahowl/aero/aerodyn_adapter.h>
#endif

using namespace chrono;

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
    ChSystemSMC system;
    system.Set_G_acc(ChVector<double>(0.0, -9.81, 0.0));
    auto solver = chrono_types::make_shared<ChSolverSparseLU>();
    system.SetSolver(solver);
    solver->UseSparsityPatternLearner(true);
    solver->LockSparsityPattern(true);

    // mesh for blade
    auto blades_mesh = chrono_types::make_shared<chrono::fea::ChMesh>();
    system.AddMesh(blades_mesh);
    // blade
    auto blade_core = get_blade_from_json((DATADIR / "blade.json").generic_string());
    // make 50 elements
    blade_core.elasto->discretization_fractions.clear();
    for (int ii = 0; ii < 51; ii++) {
        blade_core.elasto->discretization_fractions.push_back(0.02 * ii);
    }
    blade_core.build();
    blade_core.assemble(system, blades_mesh);
    auto blade = blade_core.elasto;
    blade->nodes[0]->SetFixed(true);

    system.Setup();
    system.DoStaticLinear();

    // check mass
    double blade_mass = 67051.5;
    ASSERT_NEAR(blade_mass, blade->get_mass(), 1.0);

    // check deflection from gravity (edge)
    double deflection_edge = -0.8580;
    blade->rotate(CH_C_PI, VECT_Z);
    system.DoStaticLinear();
    ASSERT_NEAR(deflection_edge, blade->nodes.back()->GetPos().y(), 0.001);

    // check deflection from gravity (flap)
    double deflection_flap = 2.91411;
    blade->rotate(CH_C_PI / 2.0, VECT_Z);
    system.DoStaticLinear();
    ASSERT_NEAR(deflection_flap, blade->nodes.back()->GetPos().y(), 0.001);
}

TEST(test_rotor, mass) {
    // system
    ChSystemSMC system;
    system.Set_G_acc(ChVector<double>(0.0, -9.81, 0.0));
    auto solver = chrono_types::make_shared<chrono::ChSolverSparseLU>();
    system.SetSolver(solver);
    solver->UseSparsityPatternLearner(true);
    solver->LockSparsityPattern(true);

    // check mass with blades
    auto blades_mesh = chrono_types::make_shared<chrono::fea::ChMesh>();
    system.AddMesh(blades_mesh);
    std::vector<std::shared_ptr<seahowl::core::Blade>> blades;
    for (int ii = 0; ii < 3; ii++) {
        auto blade_core =
            std::make_shared<seahowl::core::Blade>(get_blade_from_json((DATADIR / "blade.json").generic_string()));
        // make 50 elements
        blade_core->elasto->discretization_fractions.clear();
        for (int ii = 0; ii < 51; ii++) {
            blade_core->elasto->discretization_fractions.push_back(0.02 * ii);
        }
        blade_core->build();
        blade_core->assemble(system, blades_mesh);
        blades.push_back(blade_core);
    }

    auto rotor = get_rotor_from_json((DATADIR / "rna.json").generic_string());
    rotor.build(blades);
    rotor.assemble(system);
    rotor.elasto.body_yaw_bearing->SetBodyFixed(true);

    system.Setup();
    system.DoStaticLinear();
    // check mass
    double rotor_total_mass = 945690.92;
    ASSERT_NEAR(rotor_total_mass, rotor.elasto.get_mass(), 1.0);
}

TEST(test_tower, mass) {
    // system
    ChSystemSMC system;
    system.Set_G_acc(ChVector<double>(0.0, -9.81, 0.0));
    auto solver = chrono_types::make_shared<chrono::ChSolverMINRES>();
    system.SetSolver(solver);
    solver->EnableDiagonalPreconditioner(true);
    solver->EnableWarmStart(true);
    solver->SetMaxIterations(40000);
    solver->SetTolerance(1e-12);

    // mesh for tower
    auto tower_mesh = chrono_types::make_shared<chrono::fea::ChMesh>();
    system.AddMesh(tower_mesh);
    // tower
    auto tower = get_tower_from_json((DATADIR / "tower.json").generic_string());
    tower.build();
    tower.assemble(tower_mesh);

    system.Setup();
    system.DoStaticLinear();

    // check mass
    double tower_mass = 870391.59776;
    ASSERT_NEAR(tower_mass, tower.elasto.get_mass(), 1.0);
}

TEST(test_blade, natural_period_dynamic_edge) {
    // system
    ChSystemSMC system;
    system.Set_G_acc(ChVector<double>(0.0, -9.81, 0.0));
    auto solver = chrono_types::make_shared<chrono::ChSolverSparseLU>();
    system.SetSolver(solver);
    solver->UseSparsityPatternLearner(true);
    solver->LockSparsityPattern(true);

    // mesh for blade
    auto blades_mesh = chrono_types::make_shared<chrono::fea::ChMesh>();
    system.AddMesh(blades_mesh);
    // blade
    auto blade_core = get_blade_from_json((DATADIR / "blade.json").generic_string());
    // make 50 elements
    blade_core.elasto->discretization_fractions.clear();
    for (int ii = 0; ii < 51; ii++) {
        blade_core.elasto->discretization_fractions.push_back(0.02 * ii);
    }
    blade_core.build();
    blade_core.assemble(system, blades_mesh);
    auto blade = blade_core.elasto;
    blade->nodes[0]->SetFixed(true);

    system.Setup();
    system.DoStaticLinear();

    // static position of blade tip
    double pos0 = blade->nodes.back()->GetPos().y();

    // check zero-crossings (static position of blade tip)
    int step = 0;
    double pos_y = 0.0;
    double dt = 0.02;
    int npeaks = 0;
    double natural_period = 0.0;
    double time = 0.0;
    double end_time = 10.0;
    double start_time = 0.0;
    blade->nodes.back()->SetForce(ChVector<double>(0.0, 1000.0, 0.0));
    while (time < end_time) {
        if (time > 0.5) {
            blade->nodes.back()->SetForce(ChVector<double>(0.0, 0.0, 0.0));
            if (blade->nodes.back()->GetPos().y() < pos0 && pos_y > pos0) {
                if (start_time == 0.0) {
                    start_time = time;
                } else {
                    npeaks += 1;
                    natural_period = (time - start_time) / npeaks;
                }
            }
        }
        pos_y = blade->nodes.back()->GetPos().y();
        system.DoStepDynamics(dt);
        time += dt;
        step += 1;
    }

    // literature edgewise natural frequency for IEA15MW: 0.642Hz (1.558s)
    double natural_period_ref = 1.356;
    ASSERT_NEAR(natural_period_ref, natural_period, 0.01);
}

TEST(test_blade, natural_period_dynamic_flap) {
    // system
    ChSystemSMC system;
    system.Set_G_acc(ChVector<double>(0.0, -9.81, 0.0));
    auto solver = chrono_types::make_shared<ChSolverSparseLU>();
    system.SetSolver(solver);
    solver->UseSparsityPatternLearner(true);
    solver->LockSparsityPattern(true);

    // mesh for blade
    auto blades_mesh = chrono_types::make_shared<chrono::fea::ChMesh>();
    system.AddMesh(blades_mesh);
    // blade
    auto blade_core = get_blade_from_json((DATADIR / "blade.json").generic_string());
    // make 50 elements
    blade_core.elasto->discretization_fractions.clear();
    for (int ii = 0; ii < 51; ii++) {
        blade_core.elasto->discretization_fractions.push_back(0.02 * ii);
    }
    blade_core.build();
    blade_core.assemble(system, blades_mesh);
    auto blade = blade_core.elasto;
    blade->nodes[0]->SetFixed(true);

    // rotate blade for flap
    blade->rotate(-CH_C_PI / 2.0, VECT_Z);

    system.Setup();
    system.DoStaticLinear();

    // static position of blade tip
    double pos0 = blade->nodes.back()->GetPos().y();

    // check zero-crossings (static position of blade tip)
    int step = 0;
    double pos_y = 0.0;
    double dt = 0.02;
    int npeaks = 0;
    double natural_period = 0.0;
    double time = 0.0;
    double end_time = 10.0;
    double start_time = 0.0;
    blade->nodes.back()->SetForce(chrono::ChVector<double>(0.0, 1000.0, 0.0));
    while (time < end_time) {
        if (time > 0.5) {
            blade->nodes.back()->SetForce(chrono::ChVector<double>(0.0, 0.0, 0.0));
            if (blade->nodes.back()->GetPos().y() < pos0 && pos_y > pos0) {
                if (start_time == 0.0) {
                    start_time = time;
                } else {
                    npeaks += 1;
                    natural_period = (time - start_time) / npeaks;
                }
            }
        }
        pos_y = blade->nodes.back()->GetPos().y();
        system.DoStepDynamics(dt);
        time += dt;
        step += 1;
    }

    // literature flapwise natural frequency for IEA15MW: 0.555Hz (1.802s)
    double natural_period_ref = 1.92;
    ASSERT_NEAR(natural_period_ref, natural_period, 0.01);
}

TEST(test_turbine, rpm_initial_pitch) {
    // general options
    bool visualization_on = true;
    bool statics_prestep = true;
    // solver
    auto verbose = false;
    // timestepping
    auto timestepper_type = ChTimestepper::Type::HHT;
    double dt = 0.1;
    // wind
    auto wind_model = seahowl::aero::ConstantWind();
    wind_model.set_wind_velocity(ChVector<double>(8.0, 0.0, 0.0));
    // turbine
    double initial_pitch = CH_C_PI / 8.0;

    // system
    ChSystemSMC system;
    system.Set_G_acc(ChVector<double>(0.0, 0.0, -9.81));
    auto solver = chrono_types::make_shared<ChSolverSparseLU>();
    system.SetSolver(solver);
    solver->UseSparsityPatternLearner(true);
    solver->LockSparsityPattern(true);
    solver->SetVerbose(verbose);
    system.SetTimestepperType(ChTimestepper::Type::HHT);
    auto mystepper = std::dynamic_pointer_cast<ChTimestepperHHT>(system.GetTimestepper());
    mystepper->SetStepControl(false);
    mystepper->SetModifiedNewton(false);

    // mesh for blade
    auto blades_mesh = chrono_types::make_shared<chrono::fea::ChMesh>();
    system.AddMesh(blades_mesh);

    auto turbine_file = (DATADIR / "turbine_nocontrol.json").generic_string();
    auto turbine = get_turbine_from_json(turbine_file);

    turbine.use_aerodyn = false;

    // remove controller
    turbine.controller = std::make_shared<seahowl::servo::Controller>();
    turbine.build();
    turbine.assemble(system, blades_mesh);
    turbine.tower.elasto.nodes[0]->SetFixed(true);

    // statics
    if (statics_prestep) {
        system.DoStaticLinear();
        system.DoStaticNonlinear(10, verbose);
    }

    double time = 0.0;
    turbine.rotor.elasto.apply_collective_pitch_increment(initial_pitch);
    turbine.init(time, dt);
    // while (application.GetDevice()->run()) {
    while (time < 50) {
        // prestep
        // compute forces
        turbine.compute_wind_loads(wind_model, time);
        // prestep (accumulates loads from aero to elasto)
        turbine.prestep(time, dt);

        system.DoStepDynamics(dt);
        time += system.GetStep();

        // poststep
        turbine.poststep(time, dt);
    }

    ASSERT_NEAR(turbine.rotor.elasto.get_rpm(), 2.809, 0.02);
}

#ifdef HAVE_AERODYN
TEST(test_aerodyn, rpm_initial_pitch) {
    // general options
    bool visualization_on = true;
    bool statics_prestep = true;
    // solver
    auto verbose = false;
    // timestepping
    auto timestepper_type = ChTimestepper::Type::HHT;
    double dt = 0.1;
    // wind
    auto wind_model = seahowl::aero::ConstantWind();
    wind_model.set_wind_velocity(ChVector<double>(8.0, 0.0, 0.0));
    // turbine
    double initial_pitch = CH_C_PI / 8.0;

    // system
    ChSystemSMC system;
    system.Set_G_acc(ChVector<double>(0.0, 0.0, -9.81));
    auto solver = chrono_types::make_shared<ChSolverSparseLU>();
    system.SetSolver(solver);
    solver->UseSparsityPatternLearner(true);
    solver->LockSparsityPattern(true);
    solver->SetVerbose(verbose);
    system.SetTimestepperType(ChTimestepper::Type::HHT);
    auto mystepper = std::dynamic_pointer_cast<ChTimestepperHHT>(system.GetTimestepper());
    mystepper->SetStepControl(false);
    mystepper->SetModifiedNewton(false);

    // mesh for blade
    auto blades_mesh = chrono_types::make_shared<chrono::fea::ChMesh>();
    system.AddMesh(blades_mesh);

    auto turbine_file = (DATADIR / "turbine_nocontrol.json").generic_string();
    auto turbine = get_turbine_from_json(turbine_file);
    // remove controller
    turbine.controller = std::make_shared<seahowl::servo::Controller>();

    turbine.use_aerodyn = true;

    turbine.aerodyn = std::make_shared<seahowl::aero::AeroDynAdapter>(
        (DATADIR / "aerodyn/IEA-15-240-RWT_AeroDyn15.dat").generic_string(),
        (DATADIR / "aerodyn/IEA-15-240-RWT_InflowWind.dat").generic_string());

    turbine.build();
    turbine.assemble(system, blades_mesh);
    turbine.tower.elasto.nodes[0]->SetFixed(true);

    // statics
    if (statics_prestep) {
        system.DoStaticLinear();
        system.DoStaticNonlinear(10, verbose);
    }

    double time = 0.0;
    turbine.rotor.elasto.apply_collective_pitch_increment(initial_pitch);
    turbine.init(time, dt);
    // while (application.GetDevice()->run()) {
    while (time < 50) {
        // prestep
        // compute forces
        turbine.compute_wind_loads(wind_model, time);
        // prestep (accumulates loads from aero to elasto)
        turbine.prestep(time, dt);

        system.DoStepDynamics(dt);
        time += system.GetStep();

        // poststep
        turbine.poststep(time, dt);
    }

    ASSERT_NEAR(turbine.rotor.elasto.get_rpm(), 2.77, 0.02);
}
#endif

TEST(test_turbine, multiturbines) {
    // general options
    bool visualization_on = true;
    bool statics_prestep = true;
    // solver
    auto verbose = false;
    // timestepping
    auto timestepper_type = ChTimestepper::Type::HHT;
    double dt = 0.1;
    // wind
    auto wind_model = std::make_shared<seahowl::aero::ConstantWind>();
    wind_model->set_wind_velocity(ChVector<double>(8.0, 0.0, 0.0));
    // turbine
    double initial_pitch = CH_C_PI / 8.0;

    // system
    ChSystemSMC system_chrono;
    system_chrono.Set_G_acc(ChVector<double>(0.0, 0.0, -9.81));
    auto solver = chrono_types::make_shared<ChSolverSparseLU>();
    system_chrono.SetSolver(solver);
    solver->UseSparsityPatternLearner(true);
    solver->LockSparsityPattern(true);
    solver->SetVerbose(verbose);
    system_chrono.SetTimestepperType(ChTimestepper::Type::HHT);
    auto mystepper = std::dynamic_pointer_cast<ChTimestepperHHT>(system_chrono.GetTimestepper());
    mystepper->SetStepControl(false);
    mystepper->SetModifiedNewton(false);

    // mesh for blade
    auto blades_mesh = chrono_types::make_shared<chrono::fea::ChMesh>();
    system_chrono.AddMesh(blades_mesh);

    // system core
    seahowl::core::System system_core;
    system_core.wind_model = wind_model;

    // turbines
    auto turbine_file = (DATADIR / "turbine_nocontrol.json").generic_string();
    // turbine 1
    system_core.turbines.push_back(get_turbine_from_json(turbine_file));
    auto& turbine1 = system_core.turbines.back();
    turbine1.build();
    turbine1.assemble(system_chrono, blades_mesh);
    // turbine 2
    system_core.turbines.push_back(get_turbine_from_json(turbine_file));
    auto& turbine2 = system_core.turbines.back();
    turbine2.build();
    turbine2.assemble(system_chrono, blades_mesh);
    turbine2.translate(chrono::ChVector<double>(150.0, -150.0, 0.0));
    // turbine 3
    system_core.turbines.push_back(get_turbine_from_json(turbine_file));
    auto& turbine3 = system_core.turbines.back();
    turbine3.build();
    turbine3.assemble(system_chrono, blades_mesh);
    turbine3.translate(chrono::ChVector<double>(150.0, 150.0, 0.0));

    // remove controller
    for (auto& turbine : system_core.turbines) {
        // empty controller
        turbine.controller = std::make_shared<seahowl::servo::Controller>();
        turbine.tower.elasto.nodes.front()->SetFixed(true);
        turbine.use_aerodyn = true;
    }

    // statics
    if (statics_prestep) {
        system_chrono.DoStaticLinear();
        system_chrono.DoStaticNonlinear(10, verbose);
    }

    double time = 0.0;
    for (auto& turbine : system_core.turbines) {
        turbine.rotor.elasto.apply_collective_pitch_increment(initial_pitch);
    }
    system_core.init(time, dt);
    while (time < 50) {
        // prestep
        system_core.prestep(time, dt);

        // step
        system_chrono.DoStepDynamics(dt);
        time += system_chrono.GetStep();

        // poststep
        system_core.poststep(time, dt);
    }

    for (auto& turbine : system_core.turbines) {
        ASSERT_NEAR(turbine.rotor.elasto.get_rpm(), 2.809, 0.02);
    }
}