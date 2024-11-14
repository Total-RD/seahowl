#include <seahowl/core/simulation.h>
#include <seahowl/io/read_json.h>

#include <filesystem>  // C++17
#include <spdlog/spdlog.h>
#include <spdlog/pattern_formatter.h>
#include <map>
#include <iostream>

#include <seahowl/io/config_manager.h>

namespace fs = std::filesystem;

void run_simulation(int argc, char* argv[]) {
    spdlog::info("");
    spdlog::info("Running SEAHOWL driver.");
    // std::map to store the options
    std::map<std::string, char*> options;

    // path of main input file
    auto filepath_main = fs::path();

    if (argc > 1 && strncmp(argv[1], "-", 1) != 0) {
        filepath_main = fs::absolute(fs::path(argv[1]));
    }

    auto simulation = seahowl::core::Simulation();
    app::ConfigManager& config = simulation.getConfigManager();
    config.setJsonFilePath(filepath_main.generic_string());

    // Init
    // config.printSpec();
    config.compute(argc, argv);
    config.printCompute();

    if (filepath_main.empty()) {
        throw std::runtime_error("SEAHOWL driver: pass main input file as first argument (or type --help).");
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
