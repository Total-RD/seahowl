#include <seahowl/core/simulation.h>

#include <filesystem>  // C++17
#include <spdlog/spdlog.h>
#include <spdlog/pattern_formatter.h>

namespace fs = std::filesystem;

void run_simulation(int argc, char* argv[]) {
    spdlog::set_level(spdlog::level::info);
    spdlog::info("Start SEAHOWL simulation.");

    // path of main input file
    auto filepath_main = fs::path(u8"../data/IEA15MW/main.json");
    if (argc > 1) {
        filepath_main = fs::path(argv[1]);
    }

    auto simulation = seahowl::core::Simulation();
    simulation.populate_from_file(filepath_main.generic_string());
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
