#include "seahowl/io/output_manager.h"

#include "seahowl/io/write_csv.h"
#include "seahowl/io/write_vtk.h"
#include "seahowl/io/viz_insitu.h"
#ifdef HAVE_IRRLICHT
    #include "seahowl/io/viz_insitu_irrlicht.h"
#endif
#ifdef HAVE_AERODYN
    #include "seahowl/aero/aerodyn_adapter.h"
#endif
#include "seahowl/core/system.h"
#include "seahowl/core/turbine.h"
#include "seahowl/elasto/turbine_elasto.h"
#include "seahowl/elasto/blade_elasto.h"

#include <filesystem>  // C++17
#include <sstream>
#include <fstream>
#include <iomanip>
#include <spdlog/spdlog.h>

using namespace seahowl::io;
namespace fs = std::filesystem;

OutputManager::OutputManager(seahowl::core::System& system_core) : system_core(system_core) {}

void OutputManager::set_output_folder(const std::string& output_folder) {
    this->output_folder = output_folder;
}

void OutputManager::preinitialize() {
    if (has_vtk) {
#ifdef HAVE_AERODYN
        for (auto& turbine : system_core.turbines) {
            try {
                // set VTK options if using AeroDyn
                auto& turbine_aero = dynamic_cast<seahowl::aero::TurbineAeroDyn&>(turbine->aero);
                turbine_aero.WrVTK = 2;
                turbine_aero.WrVTK_dt = dt_output;
            } catch (const std::bad_cast& e) {
                // do nothing if not using AeroDyn
            }
        }
#endif
    }
}

void OutputManager::initialize() {
    // outputs
    spdlog::debug("Creating directory {} for outputs.", output_folder);
    fs::create_directories(output_folder);
    if (has_vtk) {
#ifdef HAVE_VTK
        output_vtk = std::make_unique<OutputSystemVTK>(system_core, output_folder + "/vtk/");
        output_vtk->initialize();
#else
        spdlog::warn("Outputs: VTK is enabled but this feature was not compiled.");
#endif
    }
    if (has_gui) {
#ifdef HAVE_IRRLICHT
        output_insitu = std::make_unique<VisualizationInSituIrrlicht>();
#else
        output_insitu = std::make_unique<VisualizationInSitu>();
        spdlog::warn("Outputs: in situ visualization is enabled but this feature was not compiled.");
#endif
        output_insitu->initialize(system_core);
        output_insitu->draw();
    }
    is_initialized = true;

    // output initial logs
    output_initial_logs();

    // output everything at step iteration 0 (creates files and CSV headers)
    output_all(0);
}

void OutputManager::output_all(int step) {
    if (!is_initialized) {
        throw std::runtime_error("Outputs: trying to output results but output manager was not initialized.");
    }
    if (has_vtk) {
#ifdef HAVE_VTK
        output_vtk->write(step);
#endif
    }
    if (has_gui) {
        output_insitu->draw();
    }

    // output
    int turbine_id = 1;
    for (auto& turbine_ptr : system_core.turbines) {
        auto& turbine = *turbine_ptr;

        // send info to logger
        std::stringstream output_sstring;
        output_sstring << "    turbine " << turbine_id << " info -> rpm: " << std::setprecision(3)
                       << turbine.rna.elasto.get_rpm() << ", power: " << turbine.get_generated_power();
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
            write_turbine_info_to_csv(output_folder + "/turbine" + std::to_string(turbine_id) + "_output", system_core);
        }

        turbine_id += 1;
    }
}

void OutputManager::output_initial_logs() {
    std::string logs_folder = output_folder + "/logs";
    spdlog::debug("Creating directory {} for logs.", logs_folder);
    fs::create_directories(logs_folder);
    // output
    int turbine_id = 1;
    for (auto& turbine_ptr : system_core.turbines) {
        auto& turbine = *turbine_ptr;

        /// @todo output tower points info from a method within TowerElasto object rather than here
        std::ofstream tower_log_reference(logs_folder + "/turbine" + std::to_string(turbine_id) +
                                          "_tower_points_reference.csv");  // Create and open the file
        std::ofstream tower_log_discretized(logs_folder + "/turbine" + std::to_string(turbine_id) +
                                            "_tower_points_discretized.csv");  // Create and open the file

        // csv header
        std::string tower_log_header =
            "fraction,density_linear,stiffness_foreaft,stiffness_sideside,stiffness_axial,stiffness_torsion,stiffness_"
            "foreaft_shear,stiffness_sideside_shear,inertia_foreaft,inertia_sideside,\n";
        tower_log_reference << tower_log_header;
        tower_log_discretized << tower_log_header;
        // loop through reference points and log their properties
        for (auto& point : turbine.elasto.tower.reference_points) {
            tower_log_reference << point.fraction << ",";
            tower_log_reference << point.density << ",";
            tower_log_reference << point.stiffness_foreaft << ",";
            tower_log_reference << point.stiffness_sideside << ",";
            tower_log_reference << point.stiffness_axial << ",";
            tower_log_reference << point.stiffness_torsion << ",";
            tower_log_reference << point.stiffness_foreaft_shear << ",";
            tower_log_reference << point.stiffness_sideside_shear << ",";
            tower_log_reference << point.inertia_foreaft << ",";
            tower_log_reference << point.inertia_sideside << ",";
            tower_log_reference << "\n";
        }
        // loop through discretized point and log their properties
        for (auto& point : turbine.elasto.tower.discretized_points) {
            tower_log_discretized << point.fraction << ",";
            tower_log_discretized << point.density << ",";
            tower_log_discretized << point.stiffness_foreaft << ",";
            tower_log_discretized << point.stiffness_sideside << ",";
            tower_log_discretized << point.stiffness_axial << ",";
            tower_log_discretized << point.stiffness_torsion << ",";
            tower_log_discretized << point.stiffness_foreaft_shear << ",";
            tower_log_discretized << point.stiffness_sideside_shear << ",";
            tower_log_discretized << point.inertia_foreaft << ",";
            tower_log_discretized << point.inertia_sideside << ",";
            tower_log_discretized << "\n";
        }

        // close files
        tower_log_reference.close();
        tower_log_discretized.close();
    }
}
