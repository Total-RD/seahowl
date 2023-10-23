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
#include "seahowl/io/read_rotor_perf.h"
#include <seahowl/io/write_csv.h>
#include <seahowl/elasto/chrono_adapters.h>

#include <seahowl/core/system.h>
#include <seahowl/aero/system_aero.h>
#include <seahowl/core/blade.h>
#include <seahowl/elasto/blade_elasto.h>
#include <seahowl/core/turbine_floating.h>

#include <filesystem>  // C++17
#include <sstream>
#include <spdlog/spdlog.h>
#include <spdlog/stopwatch.h>
#include "spdlog/pattern_formatter.h"

#include <seahowl/servo/controller.h>

using std::filesystem::path;
using std::filesystem::create_directory;
using std::filesystem::remove_all;

void output_results(seahowl::core::System& system_core) {
    // output
    auto& turbine = *system_core.turbines[0];

    // send info to logger
    std::stringstream output_sstring;
    output_sstring << "    turbine info -> rpm: " << std::setprecision(3) << turbine.rna.elasto.get_rpm()
                   << ", power: " << turbine.get_generated_power();
    int nblades = turbine.rna.blades.size();
    if (nblades <= 3) {
        for (int ii = 0; ii < turbine.rna.blades.size(); ii++) {
            output_sstring << ", pitch" << ii + 1 << ": " << turbine.rna.elasto.rotor->pitch_collective;
        }
    } else {
        output_sstring << ", pitch: " << turbine.rna.elasto.rotor->pitch_collective;
    }
    spdlog::info(output_sstring.str());

    // output info in file
    write_turbine_info_to_csv("./output/output", system_core, system_core.get_time());
}

void run_simulation(int argc, char* argv[]) {
    spdlog::set_level(spdlog::level::info);
    spdlog::info("Start SEAHOWL simulation.");

    // stopwatch before doing anything
    spdlog::stopwatch sw0;

    // SETUP
    spdlog::set_pattern("[%^%l%$] %v");
    spdlog::info("**************************************************************");
    spdlog::info("INITIAL SETUP.");
    spdlog::info("**************************************************************");

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
    spdlog::debug("Populated system.");

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
            for (auto [blade_ptr, idx_blade] = std::tuple{turbine->rna.blades.begin(), 0};
                 blade_ptr != turbine->rna.blades.end(); blade_ptr++, idx_blade++) {
                auto& blade = *blade_ptr;
                auto& post_blade =
                    vtk_outputs.emplace_back(dynamic_cast<seahowl::elasto::BladeElastoFEA&>(blade->elasto));
                post_blade.initialize(
                    ("./output/vtk/turbine" + std::to_string(idx_turbine) + "_blade" + std::to_string(idx_blade))
                        .c_str());
            }
            auto& post_tower = vtk_outputs.emplace_back(turbine->tower.elasto);
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
    if (system_core.turbines[0]->aero.use_aerodyn) {
        remove_all("./output/vtk-ADI");
    }
#endif

    // simulation loop

    // initialization
    // statics
    if (statics_prestep) {
        system_elasto->do_statics(true, 10);
        spdlog::debug("Performed statics prestep.");
    }

    system_core.initialize(system_elasto->get_time(), dt);

    int step = 0;
    output_results(system_core);
#ifdef HAVE_VTK
    if (output_vtk) {
        for (auto const& vtk_output : vtk_outputs) {
            vtk_output.write(system_elasto->get_time(), step);
        }
    }
#endif

    double time_outputs = dt_outputs;
    spdlog::info("**************************************************************");
    spdlog::info("MAIN SIMULATION LOOP.");
    spdlog::info("Initial setup time: {:.3}s.", sw0);
    spdlog::info("Resetting simulation stopwatch to 0s.");
    spdlog::info("**************************************************************");
    spdlog::stopwatch sw_total;
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
            spdlog::info("time: {:.6}s, step: {}, stopwatch: {:.3}s", system_elasto->get_time(), step, sw_total);
            output_results(system_core);
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
}

/**@brief Driver main function */
int main(int argc, char* argv[]) {
    try {
        run_simulation(argc, argv);
        return 0;
    } catch (const std::exception& e) {
        spdlog::critical(e.what());
        return 1;
    }
}
