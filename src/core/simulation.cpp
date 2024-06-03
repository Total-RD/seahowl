#include "seahowl/core/simulation.h"

#include "seahowl/core/system.h"
#include "seahowl/elasto/system_elasto.h"
#include "seahowl/elasto/blade_elasto.h"
#include "seahowl/aero/system_aero.h"
#include "seahowl/elasto/chrono_adapters.h"
#include "seahowl/io/read_json.h"
#include "seahowl/io/write_csv.h"
#include "seahowl/io/output_manager.h"

#include <fstream>
#include <filesystem>  // C++17
#include <nlohmann/json.hpp>
#include <spdlog/spdlog.h>
#include <spdlog/stopwatch.h>

using namespace seahowl::core;
namespace fs = std::filesystem;
using json = nlohmann::json;

Simulation::Simulation() {
    system_elasto = std::make_unique<seahowl::elasto::SystemElastoChrono>();
    system_aero = std::make_unique<seahowl::aero::SystemAero>();
    system_core = std::make_unique<System>(*system_elasto, *system_aero);
    outputs = std::make_unique<seahowl::io::OutputManager>(*system_core);
}

void Simulation::populate_from_file(const std::string& filepath) {
    spdlog::stopwatch sw_setup;
    spdlog::set_pattern("[%^%l%$] %v");
    spdlog::info("**************************************************************");
    spdlog::info("INITIAL SIMULATION SETUP.");
    spdlog::info("**************************************************************");

    // get main file info
    std::ifstream json_file(filepath);
    json json_obj;
    json_file >> json_obj;
    json_file.close();

    populate_system_from_json(filepath, *system_core);

    // NUMERICS options
    auto num_json = json_obj.at("numerics");
    // timestepping
    dt = num_json.at("dt").get<double>();
    duration = num_json.at("t_end").get<double>();
    // outputs
    auto outputs_json = json_obj.at("outputs");
    dt_output = outputs_json.at("dt").get<double>();
    std::string output_folder = "./output";
    if (outputs_json.contains("folder")) {
        output_folder = outputs_json.at("folder").get<std::string>();
    }

    // outputs
    outputs = std::make_unique<seahowl::io::OutputManager>(*system_core);
    outputs->set_output_folder(output_folder);
    outputs->has_vtk = outputs_json.at("VTK").get<bool>();
    outputs->has_gui = outputs_json.at("gui").get<bool>();

    spdlog::debug("Populated system in {:.3}s.", sw_setup);
}

void Simulation::initialize() {
    if (is_initialized) {
        throw std::runtime_error("Simulation was already initialized.");
    }
    outputs->initialize();
    system_core->initialize(system_core->get_time(), dt);
    is_initialized = true;
}

void Simulation::initialize_from_file(const std::string& filepath) {
    if (is_initialized) {
        throw std::runtime_error("Simulation was already initialized.");
    }
    spdlog::stopwatch sw_setup;

    // get main file info
    std::ifstream json_file(filepath);
    json json_obj;
    json_file >> json_obj;
    json_file.close();

    outputs->initialize();

    initialize_system_from_json(filepath, *system_core);
    spdlog::debug("Fully initialized system.");

    outputs->output_all(0);

    t_output_next = dt_output;
    is_initialized = true;
    spdlog::info("Initial setup time: {:.3}s.", sw_setup);
};

void Simulation::step() {
    if (!is_initialized) {
        initialize();
    }
    spdlog::stopwatch sw_step;
    // prestep
    system_core->prestep(system_core->get_time(), dt);

    // step
    system_core->step(dt);
    nstep += 1;

    // poststep
    system_core->poststep(system_core->get_time(), dt);

    // output
    if (system_core->get_time() >= (t_output_next - 1e-6)) {
        spdlog::info("time: {:.6}s, step: {}, stopwatch: {:.3}s", system_core->get_time(), nstep, sw_step);
        outputs->output_all(nstep);
        t_output_next += dt_output;
    }
}

void Simulation::run_all() {
    spdlog::info("**************************************************************");
    spdlog::info("MAIN SIMULATION LOOP.");
    spdlog::info("**************************************************************");
    spdlog::stopwatch sw_sim;
    while (system_core->get_time() < duration) {
        step();
    }
    spdlog::info("Finished simulation (runtime: {:.3}s).", sw_sim);
}
