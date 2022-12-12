
#include <chrono/fea/ChVisualizationFEAmesh.h>
#include <chrono/physics/ChBodyEasy.h>
#include <chrono/physics/ChLinkMate.h>
#include <chrono/physics/ChSystemSMC.h>
#include <chrono/solver/ChIterativeSolverLS.h>

using namespace chrono;

#ifdef HAVE_IRRLICHT
    #include <chrono_irrlicht/ChIrrApp.h>
using namespace chrono::irrlicht;
using namespace irr;
    #include <irrlicht.h>
#endif

#include <chrono/physics/ChLinkMotorRotationSpeed.h>
#include <chrono/solver/ChDirectSolverLS.h>

//#define POVRAY
#ifdef POVRAY
    #include <chrono_postprocess/ChPovRay.h>
    #include <chrono_postprocess/ChPovRayAssetCustom.h>
using namespace chrono::postprocess;
#endif

#ifdef HAVE_VTK
    #include <seahowl/io/write_vtk.h>
#endif

#include <cmath>

#include <seahowl/io/read_json.h>
#include <seahowl/io/write_csv.h>
#include <seahowl/servo/controller.h>

#ifdef HAVE_ROSCO
    #include <seahowl/servo/controller_discon.h>
#endif

#include <seahowl/elasto/blade_elasto.h>
#include <seahowl/elasto/elasto.h>
#include <seahowl/aero/wind_models.h>
#include <seahowl/core/turbine.h>
#include <seahowl/core/system.h>

#include <filesystem>  // C++17

using std::filesystem::path;
using std::filesystem::create_directory;
using std::filesystem::remove_all;

//#include <format> // c++20

/*! \mainpage SEAHOWL
 *
 * \section intro_sec Introduction
 *
 * This is the introduction.
 *
 \image html NREL_ad_driver_geom.png "source image: NREL/Openfast" width=500cm
 *
 * usage: ./seahowl_friver <data_path>
 *
 * <data path> is the location of input blade, rotar, ... sepc
 * by default ../data
 *
 * etc...
 */

/**@brief Driver main function */
int main(int argc, char* argv[]) {
    // SETUP

    auto DATADIR = absolute(path(u8"../data"));
    if (argc > 1) {
        DATADIR = absolute(path(argv[1]));
    }

    auto logoname = (DATADIR / ".." / "doc" / "source" / "totalenergies_alpha.png").generic_string();

    auto main_file = (DATADIR / "IEA15MW.json").generic_string();
    auto blade_file = (DATADIR / "IEA15MW_blade.json").generic_string();
    auto rotor_file = (DATADIR / "IEA15MW_RNA.json").generic_string();
    auto tower_file = (DATADIR / "IEA15MW_tower.json").generic_string();

    std::vector<std::string> blades_files = {blade_file, blade_file, blade_file};

// general options
#ifdef HAVE_IRRLICHT
    bool visualization_on = true;
    chrono::SetChronoDataPath(CHRONO_DATA_DIR);  // Add path to texture data
    chrono::SetChronoOutputPath(".");
#else
    bool visualization_on = false;
#endif
    bool statics_prestep = true;
    // solver
    auto solver_type = ChSolver::Type::SPARSE_LU;
    auto verbose = false;
    // timestepping
    auto timestepper_type = ChTimestepper::Type::HHT;
    double dt = 0.05;
    // system
    ChSystemSMC system;
    system.Set_G_acc(ChVector<double>(0.0, -9.81, 0.0));
    system.SetNumThreads(ChOMP::GetNumProcs(), 0, 1);
    // wind
    auto wind_model = seahowl::aero::WindRamp();
    wind_model.wind_velocity_start = ChVector<double>(12.0, 0.0, 0.0);
    wind_model.wind_velocity_stop = ChVector<double>(12.0, 0.0, 0.0);
    wind_model.direction_gravity = system.Get_G_acc().GetNormalized();
    wind_model.reference_height = 0.0;
    wind_model.time_start = 500.0;
    wind_model.time_stop = 1700;
    wind_model.shear_coefficient = 0.12;
    wind_model.reference_height = 150.0;
    // turbine
    double initial_pitch = 0.0 * CH_C_PI / 8.0;

    switch (solver_type) {
        case ChSolver::Type::SPARSE_QR: {
            std::cout << "Using SparseQR solver" << std::endl;
            auto solver = chrono_types::make_shared<ChSolverSparseQR>();
            system.SetSolver(solver);
            solver->UseSparsityPatternLearner(true);
            solver->LockSparsityPattern(true);
            solver->SetVerbose(verbose);
            break;
        }
        case ChSolver::Type::SPARSE_LU: {
            std::cout << "Using SparseLU solver" << std::endl;
            auto solver = chrono_types::make_shared<ChSolverSparseLU>();
            system.SetSolver(solver);
            solver->UseSparsityPatternLearner(true);
            solver->LockSparsityPattern(true);
            solver->SetVerbose(verbose);
            break;
        }
        case ChSolver::Type::MINRES: {
            std::cout << "Using MINRES solver" << std::endl;
            auto solver = chrono_types::make_shared<ChSolverMINRES>();
            system.SetSolver(solver);
            solver->SetMaxIterations(40000);
            solver->SetTolerance(1e-5);
            solver->EnableDiagonalPreconditioner(true);
            solver->EnableWarmStart(true);  // IMPORTANT for convergence when using EULER_IMPLICIT_LINEARIZED
            solver->SetVerbose(verbose);
            break;
        }
    }
    system.SetTimestepperType(timestepper_type);
    if (auto mystepper = std::dynamic_pointer_cast<ChTimestepperHHT>(system.GetTimestepper())) {
        mystepper->SetStepControl(false);
        mystepper->SetModifiedNewton(false);
        // mystepper->SetAlpha(-0.5);
        // GetLog() << mystepper->GetAlpha() << "\n";
    }

    // mesh for blade
    auto blades_mesh = chrono_types::make_shared<chrono::fea::ChMesh>();
    system.AddMesh(blades_mesh);

    auto seahowl_system = seahowl::core::System(seahowl::core::Turbine(get_turbine_from_main_file(main_file)), wind_model);
    auto& turbine = seahowl_system.turbine;

    // build turbine (Chrono)
    turbine.build();
    turbine.assemble(system, blades_mesh);
    turbine.tower.elasto.nodes.front()->SetFixed(true);  // foundation of the tower

    // increase elements for visualization
    for (auto& bladei : turbine.blades) {
        auto blade = bladei->elasto;
        for (auto& elm : blade->elements) {
            std::dynamic_pointer_cast<chrono::fea::ChElementBeamTaperedTimoshenko>(elm)
                ->GetTaperedSection()
                ->GetSectionA()
                ->SetDrawThickness(2.0, 0.5);
            std::dynamic_pointer_cast<chrono::fea::ChElementBeamTaperedTimoshenko>(elm)
                ->GetTaperedSection()
                ->GetSectionB()
                ->SetDrawThickness(2.0, 0.5);
        }
    }
    // increase elements for visualization
    auto& tower = turbine.tower;
    for (auto& elm : tower.elasto.elements) {
        std::dynamic_pointer_cast<chrono::fea::ChElementBeamTaperedTimoshenko>(elm)
            ->GetTaperedSection()
            ->GetSectionA()
            ->SetDrawThickness(6.0, 6.0);
        std::dynamic_pointer_cast<chrono::fea::ChElementBeamTaperedTimoshenko>(elm)
            ->GetTaperedSection()
            ->GetSectionB()
            ->SetDrawThickness(6.0, 6.0);
    }

    turbine.translate(ChVector<double>(0.0, 0.0, 0.0));
    turbine.rotate(-CH_C_PI / 2.0, VECT_X);

#ifdef POVRAY
    // Create an exporter to POVray !!!
    auto pov_exporter = ChPovRay(&system);

    // Important: set the path to the template:
    pov_exporter.SetTemplateFile(GetChronoDataFile("_template_POV.pov"));
    pov_exporter.SetBasePath(GetChronoOutputPath() + "DEMO_POVRAY");
    pov_exporter.AddAll();
    pov_exporter.ExportScript();
#endif

    // VISUALIZATION WITH IRRLICHT
#ifdef HAVE_IRRLICHT
    // Create the application UI
    ChIrrApp application(&system, L"SEAHOWL: WindTurbine", core::dimension2d<u32>(1200, 900), VerticalDir::Y, false,
                         true);

    if (visualization_on) {
        // make visualization app
        application.AddTypicalLogo(logoname);
        application.AddTypicalLights();
        application.AddTypicalSky();
        application.AddTypicalCamera(core::vector3df(-300, 150, -50));

        auto visualize_beam = chrono_types::make_shared<chrono::fea::ChVisualizationFEAmesh>(*(blades_mesh.get()));
        visualize_beam->SetFEMdataType(chrono::fea::ChVisualizationFEAmesh::E_PLOT_ELEM_BEAM_MZ);
        visualize_beam->SetColorscaleMinMax(-0.4, 0.4);
        blades_mesh->AddAsset(visualize_beam);

        // visualize nodes
        auto visualize_nodes = chrono_types::make_shared<chrono::fea::ChVisualizationFEAmesh>(*(blades_mesh.get()));
        visualize_nodes->SetFEMglyphType(chrono::fea::ChVisualizationFEAmesh::E_GLYPH_NODE_DOT_POS);
        visualize_nodes->SetFEMdataType(chrono::fea::ChVisualizationFEAmesh::E_PLOT_NODE_DISP_Y);
        visualize_nodes->SetSymbolsThickness(3.0);
        visualize_nodes->SetSymbolsScale(1.0);
        visualize_nodes->SetZbufferHide(false);
        blades_mesh->AddAsset(visualize_nodes);

        // visualize node coordinate systems
        auto visualize_nodes_coordsys =
            chrono_types::make_shared<chrono::fea::ChVisualizationFEAmesh>(*(blades_mesh.get()));
        visualize_nodes_coordsys->SetFEMglyphType(chrono::fea::ChVisualizationFEAmesh::E_GLYPH_NODE_CSYS);
        visualize_nodes_coordsys->SetFEMdataType(chrono::fea::ChVisualizationFEAmesh::E_PLOT_NONE);
        visualize_nodes_coordsys->SetSymbolsThickness(10.0);
        visualize_nodes_coordsys->SetSymbolsScale(1.0);
        visualize_nodes_coordsys->SetZbufferHide(false);
        blades_mesh->AddAsset(visualize_nodes_coordsys);

        // needed for visulization after setting everything up
        application.AssetBindAll();
        application.AssetUpdateAll();
        application.AddShadowAll();
    }
#endif
    // SIMULATION LOOP
#ifdef HAVE_IRRLICHT
    if (visualization_on) {
        application.SetTimestep(dt);
        application.SetVideoframeSave(false);
        application.SetVideoframeSaveInterval(20);
    }
#endif

    // simulation loop
    double time = 0.0;
    int step = 0;
    // initialization
    // statics
    if (statics_prestep) {
        system.DoStaticLinear();
        system.DoStaticNonlinear(10, true);
    }

    turbine.rotor.elasto.apply_collective_pitch_increment(initial_pitch);
    seahowl_system.init(time, dt);
#ifdef HAVE_ROSCO
    double omega = turbine.rotor.elasto.get_rpm() * (2 * CH_C_PI) / 60;
    // controller.init(time, dt, omega, turbine.rotor.elasto.pitch_collective, turbine.rotor.blades.size());
#endif

#ifdef HAVE_VTK
    std::vector<OutputMeshVTK> vtk_outputs;
    remove_all("./vtk");
    create_directory("./vtk");
    for (int ii = 0; ii < seahowl_system.turbine.rotor.blades.size(); ii++) {
        auto& post_blade = vtk_outputs.emplace_back(*seahowl_system.turbine.rotor.blades[ii]->elasto.get());
        post_blade.init(("./vtk/blade" + std::to_string(ii + 1)).c_str());
    }   
    auto& post_tower = vtk_outputs.emplace_back(seahowl_system.turbine.tower.elasto);
    post_tower.init("./vtk/tower");
#endif

    double torque_aero = 0.0;
    double average_torque_aero = 0.0;
    double torque_elec = 0.0;
    double average_rpm = 0.0;
    // while (application.GetDevice()->run()) {
    while (true) {
        // prestep
        // compute forces

        if (step % 20 == 0) {
            GetLog() << "time " << time << " step: " << step << " rpm: " << turbine.rotor.elasto.get_rpm() << "\n";
            write_turbine_info_to_csv("output.csv", seahowl_system, time);
        }

        seahowl_system.prestep(time, dt);
#ifdef HAVE_VTK
        if (step % 10 == 0) {
            for (auto const& vtk_output : vtk_outputs) {
                // vtk_output.write(time, step);
            }
        }

#endif

        // step
        if (visualization_on) {
#ifdef HAVE_IRRLICHT
            // this should be in while(...) loop, but it is here to allow no visualization at all
            application.GetDevice()->run();

            application.BeginScene(true, true, video::SColor(255, 140, 161, 192));
            application.DrawAll();

            // Draw also a grid on the horizontal XZ plane
            double Y0 = turbine.tower.elasto.nodes[0]->coord.pos[1];  // base (lower) position of the tower
            tools::drawGrid(application.GetVideoDriver(), 20, 20, 20, 20,
                            ChCoordsys<>(ChVector<>(0, Y0, 0), Q_from_AngX(CH_C_PI_2)),
                            video::SColor(255, 80, 100, 100), true);
            {
                auto* font = application.GetIGUIEnvironment()->getBuiltInFont();  // Font is to small

                // Write Time on UI Window
                std::string stime(1024, '\0');
                auto written = std::sprintf(&stime[0], "%.2f", time);
                stime.resize(written);
                auto sdtime = std::string("TIME: ") + stime;
                font->draw(sdtime.c_str(), core::rect<s32>(330, 10, 450, 50), video::SColor(255, 0, 0, 0));

                // Write RPM on UI Window
                std::string srpm(1024, '\0');
                written = std::sprintf(&srpm[0], "%.2f", turbine.rotor.elasto.get_rpm());
                srpm.resize(written);
                auto sdrpm = std::string("    RPM: ") + srpm;
                font->draw(sdrpm.c_str(), core::rect<s32>(450, 10, 600, 50), video::SColor(255, 0, 0, 0));

                // video::IVideoDriver* driver = application.GetDevice()->getVideoDriver();
            }

            application.DoStep();
            application.EndScene();
#endif
        } else {
            system.DoStepDynamics(dt);
        }
#ifdef POVRAY
        pov_exporter.ExportData();
#endif
        time += system.GetStep();
        step += 1;

        // poststep
        seahowl_system.poststep(time, dt);
    }

    return 0;
}
