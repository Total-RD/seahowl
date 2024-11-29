#pragma once

#include "seahowl/io/output_manager.h"
#include "seahowl/core/system.h"
#include "seahowl/aero/system_aero.h"
#include "seahowl/elasto/system_elasto.h"
#include <seahowl/io/config_manager.h>

#include <memory>
#include <string>

namespace seahowl {
namespace core {

class Simulation {
  public:
    std::unique_ptr<System> system_core;
    std::unique_ptr<seahowl::io::OutputManager> outputs;

    double dt = 0.025;
    double duration = 1000.0;
    bool is_initialized = false;

    Simulation();

    void populate_from_file(const std::string& filepath);
    void populate_from_config();
    void initialize_from_config();
    void initialize();
    void step();
    void run_all();
    seahowl::io::app::ConfigManager& getConfigManager();

  private:
    std::unique_ptr<seahowl::elasto::SystemElasto> system_elasto;
    std::unique_ptr<seahowl::aero::SystemAero> system_aero;
    int nstep = 0;
    double t_output_next = 0.0;
    std::string main_filepath;
    seahowl::io::app::ConfigManager config;
};

}  // namespace core
}  // namespace seahowl
