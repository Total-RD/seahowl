#include <seahowl/core/simulation.h>
#include <seahowl/io/read_json.h>

#include <filesystem>  // C++17
#include <spdlog/spdlog.h>
#include <spdlog/pattern_formatter.h>
#include <getopt.h>
#include <unistd.h>
#include <map>

namespace fs = std::filesystem;

void apply_args(int argc, char* argv[], std::map<std::string, char*>& options) {
    int opt;
    int option_index = 1;

    // Define long options
    static struct option long_options[] = {
        {"help", no_argument, 0, 'h'},
        {"log-level", required_argument, 0, 1007},
        {"dt", required_argument, 0, 1000},
        {"duration", required_argument, 0, 1001},
        {"dt-output", required_argument, 0, 1002},
        {"vtk", no_argument, 0, 1003},
        {"gui", no_argument, 0, 1004},
        {"output-folder", required_argument, 0, 1005},
        {"env-file", required_argument, 0, 1006},
        {0, 0, 0, 0}  // End marker
    };

    while ((opt = getopt_long(argc, argv, "h", long_options, &option_index)) != -1) {
        switch (opt) {
            case 'h':
                spdlog::info(
                    "\n"
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
                seahowl::set_log_level_global(optarg);
                options[long_options[option_index].name] = optarg;
                break;
            case 1000:
            case 1001:
            case 1002:
            case 1003:
            case 1004:
            case 1005:
            case 1006:
                spdlog::warn("Simulation variable override with {} = {}.", long_options[option_index].name, optarg);
                options[long_options[option_index].name] = optarg;
                break;
            default:
                spdlog::error("Unknown option.");
                exit(1);
        }
    }
}

void run_simulation(int argc, char* argv[]) {
    // std::map to store the options
    std::map<std::string, char*> options;

    // path of main input file
    auto filepath_main = fs::path();
    if (argc > 1 && strncmp(argv[1], "-", 1) != 0) {
        filepath_main = fs::absolute(fs::path(argv[1]));
    }

    // override values with command line arguments
    apply_args(argc, argv, options);

    if (filepath_main.empty()) {
        throw std::runtime_error("Driver: need to pass SEAHOWL main input file as first argument.");
    }

    auto simulation = seahowl::core::Simulation();

    simulation.populate_from_file(filepath_main.generic_string());

    if (options.find("output-folder") != options.end())
        simulation.outputs->set_output_folder(options["output-folder"]);

    if (options.find("log-level") != options.end())
        seahowl::set_log_level_global(options["log-level"]);

    if (options.find("dt") != options.end())
        simulation.dt = std::stod(options["dt"]);

    if (options.find("env-file") != options.end())
        populate_environmental_conditions_from_json(options["env-file"], *simulation.system_core);

    if (options.find("duration") != options.end())
        simulation.duration = std::stod(options["duration"]);

    if (options.find("dt-output") != options.end())
        simulation.outputs->dt_output = std::stod(options["dt-output"]);

    if (options.find("vtk") != options.end())
        simulation.outputs->has_vtk = true;

    if (options.find("gui") != options.end())
        simulation.outputs->has_gui = true;

    // override values with command line arguments

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
