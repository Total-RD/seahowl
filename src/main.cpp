#include <seahowl/core/simulation.h>
#include <seahowl/io/read_json.h>

#include <filesystem>  // C++17
#include <spdlog/spdlog.h>
#include <spdlog/pattern_formatter.h>
#include <getopt.h>
#include <unistd.h>

namespace fs = std::filesystem;

void apply_log_level(int argc, char* argv[]) {
    seahowl::set_log_level_global("info");

    for (int i = 1; i < argc; ++i) { 
        if (std::strcmp(argv[i], "--log-level") == 0 && i + 1 < argc) {
            seahowl::set_log_level_global(argv[i + 1]);
            break;
        }
    }
    spdlog::info("Start SEAHOWL simulation.");
}

void apply_args(int argc, char* argv[], seahowl::core::Simulation& simulation) {
    spdlog::info("Applying args.");

    int opt;
    int option_index = 1;

    // Define long options
    static struct option long_options[] = {
        {"help", no_argument, 0, 'h'},
        {"dt", required_argument, 0, 1000},
        {"duration", required_argument, 0, 1001},
        {"dt-output", required_argument, 0, 1002},
        {"vtk", no_argument, 0, 1003},
        {"gui", no_argument, 0, 1004},
        {"output-folder", required_argument, 0, 1005},
        {"env-file", required_argument, 0, 1006},
        {"log-level", required_argument, 0, 1007},
        {0, 0, 0, 0}  // End marker
    };

    while ((opt = getopt_long(argc, argv, "h", long_options, &option_index)) != -1) {
        switch (opt) {
            case 'h':
                spdlog::info(
                    "Usage: {} file_input [options]\n"
                    "Options:\n"
                    "  -h, --help            Show this help message\n"
                    "  --log-level           Set the log level critical|error|warning|info|debug|trace\n"
                    "  --dt                  Set the time step\n"
                    "  --duration            Set the duration\n"
                    "  --dt-output           Set the output time step\n"
                    "  --vtk                 Use VTK\n"
                    "  --gui                 Use GUI\n"
                    "  --output-folder       Set the output folder\n"
                    "  --env-file            Set the environment file\n",
                    argv[0]);
                exit(0);
            case 1007:
                break;   
            case 1000:
                spdlog::warn("Simulation variable override with {} = {}.", long_options[option_index].name, optarg);
                simulation.dt = std::stod(optarg);
                break;
            case 1001:
                spdlog::warn("Simulation variable override with {} = {}.", long_options[option_index].name, optarg);
                simulation.duration = std::stod(optarg);
                break;
            case 1002:
                spdlog::warn("Simulation variable override with {} = {}.", long_options[option_index].name, optarg);
                simulation.dt_output = std::stod(optarg);
                break;
            case 1003:
                simulation.outputs->has_vtk = true;
                break;
            case 1004:
                simulation.outputs->has_gui = true;
                break;
            case 1005:
                spdlog::warn("Simulation variable override with {} = {}.", long_options[option_index].name, optarg);
                simulation.outputs->set_output_folder(optarg);
                break;
            case 1006:
                spdlog::warn("Simulation variable override with {} = {}.",long_options[option_index].name, optarg);
                populate_environmental_conditions_from_json(optarg, *simulation.system_core);
                break;
            default:
                spdlog::error("Unknown option.");
                exit(1);
        }
    }

    spdlog::info("Applied args.");
}

void run_simulation(int argc, char* argv[]) {

    // apply log level
    apply_log_level(argc, argv);
    
    // path of main input file
    auto filepath_main = fs::path();
    if (argc > 1 && strncmp(argv[1], "-", 1) != 0) {
        filepath_main = fs::absolute(fs::path(argv[1]));
    }else{
        throw std::runtime_error("Driver: need to pass SEAHOWL main input file as first argument.");
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