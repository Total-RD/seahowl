#include "seahowl/io/output_manager.h"

#include "seahowl/io/write_csv.h"
#include "seahowl/io/write_vtk.h"
#include "seahowl/io/viz_insitu.h"
#ifdef HAVE_IRRLICHT
    #include "seahowl/io/viz_insitu_irrlicht.h"
#endif
#include "seahowl/core/system.h"
#include "seahowl/core/turbine.h"
#include "seahowl/elasto/turbine_elasto.h"
#include "seahowl/elasto/blade_elasto.h"

#include <filesystem>  // C++17
#include <sstream>
#include <iomanip>
#include <spdlog/spdlog.h>

using namespace seahowl::io;
namespace fs = std::filesystem;

OutputManager::OutputManager(seahowl::core::System& system_core) : system_core(system_core) {}

void OutputManager::set_output_folder(const std::string& output_folder) {
    this->output_folder = output_folder;
}

void OutputManager::initialize() {
    // outputs
    spdlog::debug("Creating directory {} for outputs.", output_folder);
    fs::create_directories(output_folder);
    if (has_vtk) {
#ifdef HAVE_VTK
        output_vtk = std::make_unique<OutputSystemVTK>(system_core, output_folder + "/vtk/");
        output_vtk->initialize();
#endif
    }
    if (has_gui) {
#ifdef HAVE_IRRLICHT
        output_insitu = std::make_unique<VisualizationInSituIrrlicht>();
#else
        output_insitu = std::make_unique<VisualizationInSitu>();
#endif
        output_insitu->initialize(system_core);
        output_insitu->draw();
    }
}

void OutputManager::output_all(int step) {
    if (has_vtk) {
#ifdef HAVE_VTK
        output_vtk->write(step);
#endif
    }
    if (has_gui) {
        output_insitu->draw();
    }

    // output
    auto& turbine = *system_core.turbines[0];

    // send info to logger
    std::stringstream output_sstring;
    output_sstring << "    turbine info -> rpm: " << std::setprecision(3) << turbine.rna.elasto.get_rpm()
                   << ", power: " << turbine.get_generated_power();
    int nblades = turbine.rna.blades.size();
    if (nblades <= 3 && nblades > 0) {
        for (int ii = 0; ii < turbine.rna.blades.size(); ii++) {
            output_sstring << ", pitch" << ii + 1 << ": " << turbine.rna.elasto.rotor->blades[ii]->pitch;
        }
    } else {
        output_sstring << ", pitch: " << turbine.rna.elasto.rotor->pitch_collective;
    }
    spdlog::info(output_sstring.str());

    if (has_csv) {
        // output info in file
        write_turbine_info_to_csv(output_folder + "/output", system_core);
    }
}
