#include <chrono/physics/ChSystemSMC.h>

#include <nlohmann/json.hpp>
using json = nlohmann::json;

#include <chrono/solver/ChDirectSolverLS.h>

#ifdef HAVE_VTK
    #include <seahowl/io/write_vtk.h>
#endif

#ifdef HAVE_IRRLICHT
    #include <chrono_irrlicht/ChVisualSystemIrrlicht.h>
    #include <seahowl/io/viz_insitu.h>
#endif

#include <cmath>

#include <seahowl/io/read_json.h>
#include <seahowl/io/write_csv.h>
#include <seahowl/elasto/chrono_adapters.h>

#include <seahowl/core/system.h>

#include <filesystem>  // C++17

using std::filesystem::path;
using std::filesystem::create_directory;
using std::filesystem::remove_all;

void output_results(seahowl::core::System& seahowl_system, chrono::ChSystemSMC& system, int step) {
    // output
    auto time = system.GetChTime();
    chrono::GetLog() << "time: " << system.GetChTime() << ", step: " << step
                     << ", rpm: " << seahowl_system.turbines[0].rotor.elasto.get_rpm()
                     << ", pitch: " << seahowl_system.turbines[0].rotor.elasto.pitch_collective << "\n";
    write_turbine_info_to_csv("./output/output.csv", seahowl_system, system.GetChTime());
}

/**@brief Driver main function */
int main(int argc, char* argv[]) {
    // SETUP

    auto DATADIR = absolute(path(u8"../data"));
    auto logoname = (DATADIR / ".." / "doc" / "source" / "totalenergies_alpha.png").generic_string();
    // outputs
    remove_all("./output");
    create_directory("./output");

    // path of main input file
    auto filepath_main = DATADIR / "IEA15MW/main.json";
    if (argc > 1) {
        filepath_main = absolute(path(argv[1]));
    }

    // system
    seahowl::elasto::SystemElastoChrono system_elasto;
    auto& system = system_elasto.chobj;
    system.SetNumThreads(chrono::ChOMP::GetNumProcs(), 0, 1);
    // mesh
    auto mesh_elasto = std::make_shared<seahowl::elasto::MeshElastoChrono>();
    system_elasto.add(mesh_elasto);

    auto seahowl_system = get_system_from_json(filepath_main.generic_string(), system_elasto, mesh_elasto);

    for (auto& turbine : seahowl_system.turbines) {
        // fix foundation of the tower
        std::dynamic_pointer_cast<seahowl::elasto::NodeElastoChrono>(turbine.tower.elasto.nodes.front())
            ->chobj->SetFixed(true);
    }

    // get main info
    std::ifstream json_file(filepath_main);
    // populate json object
    json json_obj;
    json_file >> json_obj;

    // NUMERICS options
    bool statics_prestep = json_obj.at("numerics").at("statics_prestep").get<bool>();
    // timestepping
    auto timestepper_type = chrono::ChTimestepper::Type::HHT;
    double dt = json_obj.at("numerics").at("dt").get<double>();
    double t_end = json_obj.at("numerics").at("t_end").get<double>();
    double dt_outputs = json_obj.at("outputs").at("dt").get<double>();
    bool output_vtk = json_obj.at("outputs").at("VTK").get<bool>();

    // solver
    auto solver = chrono_types::make_shared<chrono::ChSolverSparseLU>();
    system.SetSolver(solver);
    solver->UseSparsityPatternLearner(true);
    solver->LockSparsityPattern(true);
    solver->SetVerbose(false);

    // timestepper
    system.SetTimestepperType(chrono::ChTimestepper::Type::HHT);
    if (auto mystepper = std::dynamic_pointer_cast<chrono::ChTimestepperHHT>(system.GetTimestepper())) {
        mystepper->SetStepControl(false);
        mystepper->SetModifiedNewton(false);
    }

#ifdef HAVE_VTK
    std::vector<OutputMeshVTK> vtk_outputs;
    if (output_vtk) {
        for (auto [turbine_ptr, idx_turbine] = std::tuple{seahowl_system.turbines.begin(), 0};
             turbine_ptr != seahowl_system.turbines.end(); turbine_ptr++, idx_turbine++) {
            auto& turbine = *turbine_ptr;
            create_directory("./output/vtk");
            for (auto [blade_ptr, idx_blade] = std::tuple{turbine.blades.begin(), 0}; blade_ptr != turbine.blades.end();
                 blade_ptr++, idx_blade++) {
                auto& blade = *blade_ptr;
                auto& post_blade = vtk_outputs.emplace_back(*blade->elasto.get());
                post_blade.init(
                    ("./output/vtk/turbine" + std::to_string(idx_turbine) + "_blade" + std::to_string(idx_blade))
                        .c_str());
            }
            auto& post_tower = vtk_outputs.emplace_back(turbine.tower.elasto);
            post_tower.init(("./output/vtk/turbine" + std::to_string(idx_turbine) + "_tower").c_str());
        }
    }
#endif

#ifdef HAVE_IRRLICHT
    auto application = chrono_types::make_shared<chrono::irrlicht::ChVisualSystemIrrlicht>();
    application->SetWindowTitle("SEAHOWL");
    application->Initialize();
    application->SetCameraVertical(chrono::CameraVerticalDir::Z);
    application->AddLogo(logoname);
    draw_system_init(system, application);
#endif

#ifdef HAVE_AERODYN
    if (seahowl_system.turbines[0].use_aerodyn) {
        remove_all("./vtk-ADI");
    }
#endif

    // simulation loop

    // initialization
    // statics
    if (statics_prestep) {
        system.DoStaticLinear();
        system.DoStaticNonlinear(10, true);
    }

    seahowl_system.init(system.GetChTime(), dt);

    int step = 0;
    output_results(seahowl_system, system, step);
#ifdef HAVE_VTK
    if (output_vtk) {
        for (auto const& vtk_output : vtk_outputs) {
            vtk_output.write(system.GetChTime(), step);
        }
    }
#endif
    double dt_outputs_next = dt_outputs;
    while (system.GetChTime() < t_end) {
        // prestep
        seahowl_system.prestep(system.GetChTime(), dt);
        // step
        system.DoStepDynamics(dt);
        step += 1;

        // poststep
        seahowl_system.poststep(system.GetChTime(), dt);

        // output
        if (system.GetChTime() >= (dt_outputs_next - dt_outputs_next * 1e-6)) {
            output_results(seahowl_system, system, step);
#ifdef HAVE_VTK
            if (output_vtk) {
                for (auto const& vtk_output : vtk_outputs) {
                    vtk_output.write(system.GetChTime(), step);
                }
            }
#endif
#ifdef HAVE_IRRLICHT
            application->GetDevice()->run();
            draw_system(system, application);
            application->EndScene();
#endif
        }
        while (system.GetChTime() >= (dt_outputs_next - dt_outputs_next * 1e-6)) {
            dt_outputs_next += dt_outputs;
        }
    }

    return 0;
}
