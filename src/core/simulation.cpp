#include "seahowl/core/simulation.h"

#include "seahowl/core/system.h"
#include "seahowl/elasto/system_elasto.h"
#include "seahowl/elasto/blade_elasto.h"
#include "seahowl/aero/system_aero.h"
#include "seahowl/elasto/chrono_adapters.h"
#include "seahowl/io/read_json.h"
#include "seahowl/io/write_csv.h"
#include "seahowl/io/output_manager.h"
#include "seahowl/commons/utils.h"

#include <fstream>
#include <filesystem>  // C++17
#include <nlohmann/json.hpp>
#include <spdlog/spdlog.h>
#include <spdlog/stopwatch.h>

using namespace seahowl::core;
namespace fs = std::filesystem;
using json = nlohmann::json;

Simulation::Simulation()
    : config({
          "SEAHOWL_",  // envVarPrefix
          "",          // iniFilePath
          "",          // jsonFilePath
                       // variableSpecs
          {"",
           {
               {"numerics",
                {
                    {"dt", {}, "Set the time step for the simulation", "double", "0", true, true, true},
                    {"duration", {}, "Set the duration of the simulation", "double", "0", true, true, true},
                    {"statics",
                     {
                         {"linear_step", {}, "Make a linear statics step", "bool", "false", true, true, true},
                         {"nonlinear_steps", {}, "Make N nonlinear statics step", "int", "0", true, true, true},
                     },
                     ""},
                    {"presimulation",
                     {
                         {"dt", {}, "Set the time step for the presimulation", "double", "0", true, true, true},
                         {"duration", {}, "Set the duration of the presimulation", "double", "0", true, true, true},
                         {"presetup", {}, "Include presetup during presimulation", "bool", "true", true, true, true},
                         {"fix_towers", {}, "Include presetup during presimulation", "bool", "true", true, true, true},
                     },
                     ""},
                },
                ""},
               {"outputs",
                {
                    {"dt", {}, "Set the time step for generating outputs", "double", "0", true, true, true},
                    {"folder", {}, "Set the path of the folder for outputs", "string", "./output", true, true, true},
                    {"VTK", {}, "Generate VTK outputs", "bool", "true", true, true, true},
                    {"log_level",
                     {},
                     "Set the log level (critical|error|warning|info|debug|trace)",
                     "string",
                     "default",
                     true,
                     true,
                     true},
                    {"gui", {}, "Display GUI (in situ visualization)", "bool", "true", true, true, true},
                },
                ""},
               {"environment",
                {
                    {"file", {}, "Set the environmental conditions file", "path", "", true, true, true},
                },
                ""},
           }},
      }) {
    system_elasto = std::make_unique<seahowl::elasto::SystemElastoChrono>();
    system_aero = std::make_unique<seahowl::aero::SystemAero>();
    system_core = std::make_unique<System>(*system_elasto, *system_aero);
    outputs = std::make_unique<seahowl::io::OutputManager>(*system_core);
}

void Simulation::populate_from_file(const std::string& filepath) {
    config.set_json_filepath(filepath);
    config.compute();
    populate_from_config();
}

void Simulation::populate_from_config() {
    spdlog::stopwatch sw_setup;
    auto filepath = config.get_json_filepath();

    // timestepping
    dt = config.get_double("numerics.dt");
    duration = config.get_double("numerics.duration");

    // outputs
    if (!seahowl::LOG_LEVEL_SET) {
        // logging
        auto log_level = config.get_string("outputs.log_level");
        seahowl::set_log_level_global(log_level);
    }

    // output manager
    outputs->dt_output = config.get_double("outputs.dt");
    outputs->set_output_folder(config.get_string("outputs.folder"));
    outputs->has_vtk = config.get_bool("outputs.VTK");
    outputs->has_gui = config.get_bool("outputs.gui");

    spdlog::info("");
    spdlog::info("-------------------------------------------------");
    spdlog::info("INITIAL SIMULATION SETUP");
    spdlog::info("-------------------------------------------------");
    spdlog::info("");

    seahowl::io::populate_system_from_config(config, *system_core);

    spdlog::debug("Populated system in {:.3}s.", sw_setup);
}

void Simulation::initialize() {
    if (is_initialized) {
        throw std::runtime_error("Simulation was already initialized.");
    }
    spdlog::stopwatch sw_setup;

    // pre-initialize outputs (sets output-related variable needed in system before initializing)
    outputs->preinitialize();

    // initialize system
    system_core->initialize(system_core->get_time(), dt);

    // initialize outputs (after initializing everything in system)
    outputs->initialize();

    t_output_next = outputs->dt_output;
    is_initialized = true;
    spdlog::info("Simulation initialized in {:.3}s.", sw_setup);
}

void Simulation::initialize_from_config() {
    initialize();

    // initialization
    // statics
    auto linear_step = config.get_bool("numerics.statics.linear_step");
    auto nonlinear_steps = config.get_int("numerics.statics.nonlinear_steps");
    if (linear_step && nonlinear_steps > 0) {
        system_core->elasto.do_statics(linear_step, nonlinear_steps);
        system_core->poststep(system_core->get_time(), dt);  // poststep to update positions aero
    }

    // apply presetup
    auto presimulation_duration = config.get_double("numerics.presimulation.duration");
    if (presimulation_duration > 0) {
        auto presimulation_dt = config.get_double("numerics.presimulation.dt");
        auto do_presetup = config.get_bool("numerics.presimulation.presetup");
        auto fix_towers = config.get_bool("numerics.presimulation.fix_towers");
        system_core->run_presimulation(presimulation_duration, presimulation_dt, fix_towers, do_presetup);
    }
};

void Simulation::step() {
    if (!is_initialized) {
        initialize();
    }
    if (dt <= 0.0) {
        throw std::runtime_error("Cannot make a simulation simulation with dt = " + std::to_string(dt) + ".");
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
        t_output_next += outputs->dt_output;
    } else {
        spdlog::debug("time: {:.6}s, step: {}, stopwatch: {:.3}s", system_core->get_time(), nstep, sw_step);
    }
}

void Simulation::run_all() {
    spdlog::info("");
    spdlog::info("-------------------------------------------------");
    spdlog::info("MAIN SIMULATION LOOP");
    spdlog::info("-------------------------------------------------");
    spdlog::info("");

    if (duration <= 0.0) {
        throw std::runtime_error("Cannot run a simulation with duration = " + std::to_string(duration) + ".");
    }

    spdlog::stopwatch sw_sim;
    while (system_core->get_time() < duration) {
        step();
    }
    spdlog::info("Finished simulation (runtime: {:.3}s).", sw_sim);
    spdlog::info("-------------------------------------------------");
}

seahowl::io::app::ConfigManager& Simulation::getConfigManager() {
    return config;
}
