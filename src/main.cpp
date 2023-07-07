#include <chrono/physics/ChSystemSMC.h>
#include <chrono/fea/ChNodeFEAxyzrot.h>

#include <nlohmann/json.hpp>
using json = nlohmann::json;

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

void output_results(seahowl::core::System& system_core, int step) {
    // output
    auto& turbine = system_core.turbines[0];
    std::cout << "time: " << system_core.get_time() << ", step: " << step
              << ", rpm: " << turbine.rotor.elasto.get_rpm();
    int nblades = turbine.rotor.blades.size();
    if (nblades <= 3) {
        for (int ii = 0; ii < turbine.rotor.blades.size(); ii++) {
            std::cout << ", pitch" << ii << ": " << turbine.rotor.blades[ii]->elasto.pitch;
        }
    } else {
        std::cout << ", pitch: " << turbine.rotor.elasto.pitch_collective;
    }
    std::cout << std::endl;
    write_turbine_info_to_csv("./output/output", system_core, system_core.get_time());
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

    // system elasto
    auto system_elasto = std::make_shared<seahowl::elasto::SystemElastoChrono>();
    auto system_chrono = system_elasto->chobj;
    system_chrono->SetNumThreads(chrono::ChOMP::GetNumProcs(), 0, 1);
    // system aero
    auto system_aero = std::make_shared<seahowl::aero::SystemAero>();
    // system core
    auto system_core = seahowl::core::System();
    system_core.system_elasto = system_elasto;
    system_core.system_aero = system_aero;
    populate_system_from_json(filepath_main.generic_string(), system_core);

    for (auto& turbine : system_core.turbines) {
        // fix foundation of the tower
        turbine.tower.elasto.nodes.front()->set_fixed(true);
    }

    // get main info
    std::ifstream json_file(filepath_main);
    // populate json object
    json json_obj;
    json_file >> json_obj;

    // NUMERICS options
    bool statics_prestep = json_obj.at("numerics").at("statics_prestep").get<bool>();
    // timestepping
    double dt = json_obj.at("numerics").at("dt").get<double>();
    double t_end = json_obj.at("numerics").at("t_end").get<double>();
    double dt_outputs = json_obj.at("outputs").at("dt").get<double>();
    bool output_vtk = json_obj.at("outputs").at("VTK").get<bool>();

#ifdef HAVE_VTK
    std::vector<OutputMeshVTK> vtk_outputs;
    if (output_vtk) {
        for (auto [turbine_ptr, idx_turbine] = std::tuple{system_core.turbines.begin(), 0};
             turbine_ptr != system_core.turbines.end(); turbine_ptr++, idx_turbine++) {
            auto& turbine = *turbine_ptr;
            create_directory("./output/vtk");
            for (auto [blade_ptr, idx_blade] = std::tuple{turbine.rotor.blades.begin(), 0};
                 blade_ptr != turbine.rotor.blades.end(); blade_ptr++, idx_blade++) {
                auto& blade = *blade_ptr;
                auto& post_blade = vtk_outputs.emplace_back(blade->elasto);
                post_blade.initialize(
                    ("./output/vtk/turbine" + std::to_string(idx_turbine) + "_blade" + std::to_string(idx_blade))
                        .c_str());
            }
            auto& post_tower = vtk_outputs.emplace_back(turbine.tower.elasto);
            post_tower.initialize(("./output/vtk/turbine" + std::to_string(idx_turbine) + "_tower").c_str());
        }
    }
#endif

#ifdef HAVE_IRRLICHT
    auto application = chrono_types::make_shared<chrono::irrlicht::ChVisualSystemIrrlicht>();
    application->SetWindowTitle("SEAHOWL");
    application->Initialize();
    application->SetCameraVertical(chrono::CameraVerticalDir::Z);
    application->AddLogo(logoname);
    draw_system_init(system_chrono, application);
#endif

#ifdef HAVE_AERODYN
    if (system_core.turbines[0].aero.use_aerodyn) {
        remove_all("./output/vtk-ADI");
    }
#endif

    // simulation loop

    // initialization
    // statics
    if (statics_prestep) {
        system_elasto->do_statics(true, 10);
    }

    system_core.initialize(system_elasto->get_time(), dt);

    int step = 0;
    output_results(system_core, step);
#ifdef HAVE_VTK
    if (output_vtk) {
        for (auto const& vtk_output : vtk_outputs) {
            vtk_output.write(system_elasto->get_time(), step);
        }
    }
#endif
    double time_outputs = dt_outputs;
    while (system_elasto->get_time() < t_end) {
        // prestep
        system_core.prestep(system_core.get_time(), dt);
        // step
        system_core.step(dt);
        step += 1;

        // poststep
        system_core.poststep(system_core.get_time(), dt);

        // output
        if (system_core.get_time() >= (time_outputs - 1e-6)) {
            output_results(system_core, step);
#ifdef HAVE_VTK
            if (output_vtk) {
                for (auto const& vtk_output : vtk_outputs) {
                    vtk_output.write(system_core.get_time(), step);
                }
            }
#endif
#ifdef HAVE_IRRLICHT
            application->GetDevice()->run();
            draw_system(system_chrono, application);
            application->EndScene();
#endif
            time_outputs += dt_outputs;
        }
    }

    return 0;
}
