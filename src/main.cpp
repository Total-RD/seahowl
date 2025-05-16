#include <seahowl/core/simulation.h>
#include <seahowl/io/read_input.h>
#include <seahowl/io/config_manager.h>

#include <filesystem>  // C++17
#include <spdlog/spdlog.h>
#include <spdlog/pattern_formatter.h>
#include <map>
#include <iostream>

namespace fs = std::filesystem;

void run_simulation(int argc, char* argv[]) {
    spdlog::info("");
    spdlog::info("Running SEAHOWL driver.");
    // std::map to store the options
    std::map<std::string, char*> options;

    // path of main input file
    std::string filepath_main;

    if (argc > 1 && strncmp(argv[1], "-", 1) != 0) {
        // check if first argument is an option or the main input file
        filepath_main = argv[1];
        if (filepath_main.empty() || !fs::is_regular_file(filepath_main)) {
            // check if main input file exists
            throw std::runtime_error("SEAHOWL driver: main input file not found: " + filepath_main +
                                     " (absolute: " + fs::absolute(filepath_main).generic_string() + ").");
        }
    }

    auto simulation = seahowl::core::Simulation();
    seahowl::io::app::ConfigManager& config = simulation.getConfigManager();
    config.set_json_filepath(filepath_main);

    // Init
    // config.printSpec();
    config.compute(argc, argv);
    config.print_compute();

    if (filepath_main.empty()) {
        throw std::runtime_error("SEAHOWL driver: pass main input file as first argument.");
    }

    simulation.populate_from_config();

    simulation.initialize_from_config();

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
