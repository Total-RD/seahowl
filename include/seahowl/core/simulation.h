#pragma once

#include "seahowl/io/output_manager.h"
#include "seahowl/core/system.h"
#include "seahowl/aero/system_aero.h"
#include "seahowl/elasto/system_elasto.h"

#include <memory>
#include <string>

namespace seahowl {
namespace core {

class Simulation {
  public:
    std::unique_ptr<System> system_core;

    double dt = 0.025;
    double dt_output = 0.;
    double duration = 1000.0;

    bool output_vtk = false;
    std::string output_folder;

    Simulation();

    void initialize_from_file(const std::string& filepath);
    void step();
    void run_all();

  private:
    std::unique_ptr<seahowl::elasto::SystemElasto> system_elasto;
    std::unique_ptr<seahowl::aero::SystemAero> system_aero;
    int nstep = 0;
    double t_output_next = 0.0;
    std::unique_ptr<seahowl::io::OutputManager> outputs;
};

}  // namespace core
}  // namespace seahowl
