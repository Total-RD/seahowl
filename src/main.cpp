#include <nlohmann/json.hpp>
using json = nlohmann::json;

#ifdef HAVE_VTK
    #include <seahowl/io/write_vtk.h>
#endif

#include <cmath>

#include <seahowl/io/read_json.h>
#include <seahowl/io/write_csv.h>
#include <seahowl/io/viz_insitu.h>
#ifdef HAVE_IRRLICHT
    #include <seahowl/io/viz_insitu_irrlicht.h>
#endif

#include <seahowl/core/system.h>
#include <seahowl/aero/system_aero.h>
#include <seahowl/core/blade.h>
#include <seahowl/elasto/blade_elasto.h>
#include <seahowl/core/turbine_floating.h>
#include <seahowl/elasto/mooring_elasto.h>

#include <filesystem>  // C++17
#include <fstream>
#include <sstream>
#include <spdlog/spdlog.h>
#include <spdlog/stopwatch.h>
#include <spdlog/pattern_formatter.h>

namespace fs = std::filesystem;
using std::filesystem::path;
using std::filesystem::create_directory;
using std::filesystem::remove_all;

void output_results(const seahowl::core::System& system_core, const std::string& output_folder) {
    // output
    auto& turbine = *system_core.turbines[0];

    // send info to logger
    std::stringstream output_sstring;
    output_sstring << "    turbine info -> rpm: " << std::setprecision(3) << turbine.rna.elasto.get_rpm()
                   << ", power: " << turbine.get_generated_power();
    int nblades = turbine.rna.blades.size();
    if (nblades <= 3 && nblades > 0) {
        for (int ii = 0; ii < turbine.rna.blades.size(); ii++) {
            output_sstring << ", pitch" << ii + 1 << ": " << turbine.rna.elasto.rotor->blades[ii]->pitch;
        }
    } else {
        output_sstring << ", pitch: " << turbine.rna.elasto.rotor->pitch_collective;
    }
    spdlog::info(output_sstring.str());

    // output info in file
    write_turbine_info_to_csv(output_folder + "/output", system_core);
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

    // path of main input file
    auto filepath_main = DATADIR / "IEA15MW/main.json";
    if (argc > 1) {
        filepath_main = absolute(path(argv[1]));
    }
    // get main file info
    std::ifstream json_file(filepath_main.generic_string());
    json json_obj;
    json_file >> json_obj;
    json_file.close();

    // NUMERICS options
    auto num_json = json_obj.at("numerics");
    // timestepping
    auto dt = num_json.at("dt").get<double>();
    auto t_end = num_json.at("t_end").get<double>();
    // outputs
    auto outputs_json = json_obj.at("outputs");
    auto dt_outputs = outputs_json.at("dt").get<double>();
    auto output_vtk = outputs_json.at("VTK").get<bool>();
    auto has_gui = outputs_json.at("gui").get<bool>();
    std::string output_folder = "./output";
    if (outputs_json.contains("folder")) {
        output_folder = outputs_json.at("folder").get<std::string>();
    }

    // outputs
    fs::create_directories(output_folder);

    // system elasto
    auto system_elasto = seahowl::elasto::SystemElastoChrono();
    // system aero
    auto system_aero = seahowl::aero::SystemAero();
    // system core
    auto system_core = seahowl::core::System(system_elasto, system_aero);

    populate_system_from_json(filepath_main.generic_string(), system_core);
    spdlog::debug("Populated system.");

    initialize_system_from_json(filepath_main.generic_string(), system_core);
    spdlog::debug("Fully initialized system.");

#ifdef HAVE_VTK
    auto vtk_system = seahowl::io::OutputSystemVTK(system_core, output_folder + "/vtk/");
    if (output_vtk) {
        vtk_system.initialize();
    }
#endif

    std::unique_ptr<VisualizationInSitu> viz_insitu;
    if (has_gui) {
#ifdef HAVE_IRRLICHT
        viz_insitu = std::make_unique<VisualizationInSituIrrlicht>();
#else
        viz_insitu = std::make_unique<VisualizationInSitu>();
#endif
        viz_insitu->initialize(system_core);
        viz_insitu->draw();
    }

    int step = 0;
    output_results(system_core, output_folder);
#ifdef HAVE_VTK
    if (output_vtk) {
        vtk_system.write(step);
    }
#endif

    double time_outputs = dt_outputs;
    spdlog::info("**************************************************************");
    spdlog::info("MAIN SIMULATION LOOP.");
    spdlog::info("Initial setup time: {:.3}s.", sw0);
    spdlog::info("Resetting simulation stopwatch to 0s.");
    spdlog::info("**************************************************************");
    spdlog::stopwatch sw_total;
    while (system_core.get_time() < t_end) {
        // prestep
        system_core.prestep(system_core.get_time(), dt);

        // step
        system_core.step(dt);
        step += 1;

        // poststep
        system_core.poststep(system_core.get_time(), dt);

        // output
        if (system_core.get_time() >= (time_outputs - 1e-6)) {
            spdlog::info("time: {:.6}s, step: {}, stopwatch: {:.3}s", system_core.get_time(), step, sw_total);
            output_results(system_core, output_folder);
#ifdef HAVE_VTK
            if (output_vtk) {
                vtk_system.write(step);
            }
#endif
            if (has_gui) {
                viz_insitu->draw();
            }
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
