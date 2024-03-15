#include <seahowl/core/simulation.h>

#include <filesystem>  // C++17
#include <spdlog/spdlog.h>
#include <spdlog/pattern_formatter.h>

namespace fs = std::filesystem;

void apply_log_level(int argc, char* argv[]) {
    seahowl::set_log_level_global("info");
    for (int ii = 2; ii < argc; ++ii) {
        std::string arg = argv[ii];
        if (arg == "--log-level") {
            if (ii < argc - 1) {
                std::string log_level = argv[ii + 1];
                seahowl::set_log_level_global(log_level);
            } else {
                throw std::runtime_error("No value following " + std::string(argv[ii]) + " argument.");
            }
        }
    }
    spdlog::info("Start SEAHOWL simulation.");
}

void apply_args(int argc, char* argv[], seahowl::core::Simulation& simulation) {
    spdlog::info("Applying args.");
    for (int ii = 2; ii < argc; ++ii) {
        std::string arg = argv[ii];
        if (arg == "--dt") {
            if (ii < argc - 1) {
                double dt = std::stod(argv[ii + 1]);
                spdlog::warn("Simulation variable override with {} {}.", argv[ii], dt);
                simulation.dt = dt;
            } else {
                throw std::runtime_error("No value following " + std::string(argv[ii]) + " argument.");
            }
        } else if (arg == "--duration") {
            spdlog::info("duration");
            if (ii < argc - 1) {
                double duration = std::stod(argv[ii + 1]);
                spdlog::warn("Simulation variable override with {} {}.", argv[ii], duration);
                simulation.duration = duration;
            } else {
                throw std::runtime_error("No value following " + std::string(argv[ii]) + " argument.");
            }
        } else if (arg == "--dt-output") {
            if (ii < argc - 1) {
                double dt_output = std::stod(argv[ii + 1]);
                spdlog::warn("Simulation variable override with {} {}.", argv[ii], dt_output);
                simulation.dt_output = dt_output;
            } else {
                throw std::runtime_error("No value following " + std::string(argv[ii]) + " argument.");
            }
        } else if (arg == "--vtk") {
            simulation.outputs->has_vtk = true;
        } else if (arg == "--novtk") {
            simulation.outputs->has_vtk = false;
        } else if (arg == "--gui") {
            simulation.outputs->has_gui = true;
        } else if (arg == "--nogui") {
            simulation.outputs->has_gui = false;
        } else if (arg == "--output-folder") {
            if (ii < argc - 1) {
                std::string output_folder = argv[ii + 1];
                spdlog::warn("Simulation variable override with {} {}.", argv[ii], output_folder);
                simulation.outputs->set_output_folder(output_folder);
            } else {
                throw std::runtime_error("No value following " + std::string(argv[ii]) + " argument.");
            }
        }
    }
    spdlog::info("Applied args.");
}

void run_simulation(int argc, char* argv[]) {
    // apply log level
    apply_log_level(argc, argv);

    // path of main input file
    auto filepath_main = fs::path(u8"../data/IEA15MW/main.json");
    if (argc > 1) {
        filepath_main = fs::path(argv[1]);
    }

    auto simulation = seahowl::core::Simulation();
    simulation.populate_from_file(filepath_main.generic_string());

    // override values with command line arguments
    apply_args(argc, argv, simulation);

    simulation.initialize_from_file(filepath_main.generic_string());

    simulation.run_all();
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
