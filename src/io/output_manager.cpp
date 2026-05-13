// SPDX-License-Identifier: Apache-2.0
// Copyright 2022–2026 TotalEnergies
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//

#include "seahowl/io/output_manager.h"

// SEAHOWL headers
#include "seahowl/core/blade.h"
#include "seahowl/core/floater.h"
#include "seahowl/core/monopile.h"
#include "seahowl/core/system.h"
#include "seahowl/core/turbine.h"
#include "seahowl/elasto/blade_elasto.h"
#include "seahowl/elasto/floater_elasto.h"
#include "seahowl/elasto/rotor_elasto.h"
#include "seahowl/elasto/tower_elasto.h"
#include "seahowl/elasto/turbine_elasto.h"
#include "seahowl/env/env_model.h"
#include "seahowl/env/fluid_models.h"
#include "seahowl/env/wind_models.h"
#include "seahowl/fluid/aero/blade_aero.h"
#include "seahowl/io/viz_insitu.h"
#include "seahowl/io/write_csv.h"
#include "seahowl/io/write_vtk.h"
#include "seahowl/servo/controller.h"
#ifdef HAVE_IRRLICHT
    #include "seahowl/io/viz_insitu_irrlicht.h"
#endif
#ifdef HAVE_AERODYN
    #include "seahowl/fluid/aero/aerodyn_adapter.h"
#endif

// Third-party libraries
#include <spdlog/spdlog.h>

// Standard library
#include <filesystem>
#include <fstream>
#include <iomanip>
#include <sstream>

using seahowl::PI;
using namespace seahowl::io;
namespace fs = std::filesystem;

/**
 * @brief Adds functions for outputting basic infos of a turbine.
 *
 * @param[in] custom_csv CustomCSV to which info is outputted.
 * @param[in] turbine Turbine class from which variables are outputted.
 */
void add_basic_turbine_info_to_csv(seahowl::io::CustomCSV& custom_csv, seahowl::core::Turbine& turbine) {
    custom_csv.add_function("rpm [-]", [&turbine]() { return turbine.rna.elasto.get_rpm(); });
    custom_csv.add_function("power [W]", [&turbine]() { return turbine.get_generated_power(); });
    custom_csv.add_function("pitch collective [rad]",
                            [&turbine]() { return turbine.rna.elasto.rotor->pitch_collective; });
    custom_csv.add_function("torque elec [Nm]", [&turbine]() { return turbine.controller->get_torque_elec(); });
    custom_csv.add_function("axial thrust [N]", [&turbine]() { return turbine.rna.elasto.get_axial_thrust(); });
    custom_csv.add_function("axial torque [Nm]", [&turbine]() { return turbine.rna.elasto.get_axial_torque(); });
    custom_csv.add_function("rotor azimuth [rad]", [&turbine]() { return turbine.rna.elasto.get_azimuth(); });
    custom_csv.add_function("tower base moment [Nm]",
                            [&turbine]() { return turbine.tower.elasto.get_tower_base_moment(); });
    custom_csv.add_function("tower base force [N]",
                            [&turbine]() { return turbine.tower.elasto.get_tower_base_force(); });
    custom_csv.add_function("tower top moment [Nm]",
                            [&turbine]() { return turbine.tower.elasto.get_tower_top_moment(); });
    custom_csv.add_function("tower top force [N]", [&turbine]() { return turbine.tower.elasto.get_tower_top_force(); });
    for (size_t idx_blade = 0; idx_blade < turbine.rna.rotor.blades.size(); idx_blade++) {
        auto& blade = *turbine.rna.rotor.blades[idx_blade];
        custom_csv.add_function("blade" + std::to_string(idx_blade + 1) + " root moment [Nm]",
                                [&blade]() { return blade.elasto.get_blade_root_moment(); });
        custom_csv.add_function("blade" + std::to_string(idx_blade + 1) + " azimuth [rad]", [&blade, &turbine]() {
            // check that blade_azimuth is between pi and -pi
            auto blade_azimuth = blade.elasto.azimuth0 + turbine.rna.elasto.get_azimuth();
            if (blade_azimuth < -PI || blade_azimuth > PI) {
                blade_azimuth = abs(std::fmod((blade_azimuth + 3 * PI), 2 * PI)) - PI;
            }
            return blade_azimuth;
        });
        custom_csv.add_function("blade" + std::to_string(idx_blade + 1) + " pitch [rad]",
                                [&blade]() { return blade.elasto.get_pitch(); });
    }
    if (turbine.foundation) {
        try {  // check if floater
            auto& floater = dynamic_cast<seahowl::core::Floater&>(*turbine.foundation);
            auto& floater_elasto = floater.elasto;
            custom_csv.add_function("floater position [m]",
                                    [&floater_elasto]() { return floater_elasto.body_main->get_position(); });
            custom_csv.add_function("floater rotation [rad]",
                                    [&floater_elasto]() { return floater_elasto.body_main->get_rpy_angles(); });
            for (size_t idx_mooring = 0; idx_mooring < floater_elasto.mooring_system->moorings.size(); idx_mooring++) {
                auto& mooring = *floater_elasto.mooring_system->moorings[idx_mooring];
                custom_csv.add_function("fairlead" + std::to_string(idx_mooring + 1) + " tension [N]",
                                        [&mooring]() { return mooring.get_tension_fairlead(); });
            }
        } catch (const std::bad_cast& e) {
            // do nothing if no floater
        }
        try {  // check if monopile
            auto& monopile = dynamic_cast<seahowl::core::Monopile&>(*turbine.foundation);
            auto& monopile_elasto = monopile.elasto;
            custom_csv.add_function("monopile base moment [Nm]",
                                    [&monopile_elasto]() { return monopile_elasto.get_tower_base_moment(); });
            custom_csv.add_function("monopile base force [N]",
                                    [&monopile_elasto]() { return monopile_elasto.get_tower_base_force(); });
        } catch (const std::bad_cast& e) {
            // do nothing if no monopile
        }
    }
}

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
                auto& turbine_aero = dynamic_cast<seahowl::aero::TurbineAeroDyn&>(turbine->fluid);
                turbine_aero.WrVTK = 2;
                turbine_aero.WrVTK_dt = dt_output;
            } catch (const std::bad_cast& e) {
                // do nothing if not using AeroDyn
            }
        }
#endif
    }
    for (auto& turbine : system_core.turbines) {
        if (turbine->controller) {
            turbine->controller->output_folder = output_folder;
        }
    }
}

void OutputManager::initialize() {
    // outputs
    if (output_folder.empty()) {
        output_folder = fs::current_path().generic_string();
    }
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

    if (has_csv) {
        for (int idx_turbine = 0; idx_turbine < system_core.turbines.size(); idx_turbine++) {
            std::string csv_path = "turbine" + std::to_string(idx_turbine + 1) + "_output.csv";
            auto& custom_csv = create_new_csv(csv_path);
            auto& system_core = this->system_core;
            auto& turbine = *system_core.turbines[idx_turbine];
            custom_csv.add_function("time [s]", [system_core]() { return system_core.get_time(); });
            if (system_core.env_model->fluid_models.has_model()) {
                custom_csv.add_function("wind speed hub [m/s]", [&system_core, &turbine]() {
                    return system_core.env_model->fluid_models.get_velocity(
                        turbine.rna.elasto.rotor->body_hub->get_position(), system_core.get_time());
                });
            }
            add_basic_turbine_info_to_csv(custom_csv, turbine);
        }
    }

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
    // output info in CSV file if any CustomCSV was created
    for (auto& custom_csv : custom_csv_list) {
        custom_csv.write_row();
    }

    // output
    int turbine_id = 1;
    for (auto& turbine_ptr : system_core.turbines) {
        auto& turbine = *turbine_ptr;

        // send info to logger
        std::stringstream output_sstring;
        output_sstring << "    turbine " << turbine_id << " info -> rpm: " << std::setprecision(3)
                       << turbine.rna.elasto.get_rpm() << ", power: " << turbine.get_generated_power()
                       << ", yaw: " << turbine.rna.elasto.get_yaw();
        int nblades = turbine.rna.rotor.blades.size();
        if (nblades <= 3 && nblades > 0) {
            for (int ii = 0; ii < turbine.rna.rotor.blades.size(); ii++) {
                output_sstring << ", pitch" << ii + 1 << ": " << turbine.rna.elasto.rotor->blades[ii]->get_pitch();
            }
        } else {
            output_sstring << ", pitch: " << turbine.rna.elasto.rotor->pitch_collective;
        }
        spdlog::info(output_sstring.str());

        turbine_id += 1;
    }
}

void OutputManager::output_initial_logs() {
    std::string logs_folder = (fs::path(output_folder) / "logs").generic_string();
    spdlog::debug("Creating initial logs in {}.", logs_folder);
    fs::create_directories(logs_folder);
    // output
    for (int idx_turbine = 0; idx_turbine < system_core.turbines.size(); idx_turbine++) {
        auto& turbine = *system_core.turbines[idx_turbine];

        // reference points
        size_t idx_point;
        auto& reference_points = turbine.elasto.tower->reference_points;
        auto csv_out = seahowl::io::CustomCSV(
            (fs::path(logs_folder) / ("turbine" + std::to_string(idx_turbine + 1) + "_tower_points_reference.csv"))
                .generic_string());
        csv_out.add_function("fraction",
                             [&reference_points, &idx_point]() { return reference_points[idx_point].fraction; });
        csv_out.add_function("density_linear",
                             [&reference_points, &idx_point]() { return reference_points[idx_point].density; });
        csv_out.add_function("stiffness_foreaft", [&reference_points, &idx_point]() {
            return reference_points[idx_point].stiffness_foreaft;
        });
        csv_out.add_function("stiffness_sideside", [&reference_points, &idx_point]() {
            return reference_points[idx_point].stiffness_sideside;
        });
        csv_out.add_function("stiffness_axial",
                             [&reference_points, &idx_point]() { return reference_points[idx_point].stiffness_axial; });
        csv_out.add_function("stiffness_torsion", [&reference_points, &idx_point]() {
            return reference_points[idx_point].stiffness_torsion;
        });
        csv_out.add_function("stiffness_foreaft_shear", [&reference_points, &idx_point]() {
            return reference_points[idx_point].stiffness_foreaft_shear;
        });
        csv_out.add_function("stiffness_sideside_shear", [&reference_points, &idx_point]() {
            return reference_points[idx_point].stiffness_sideside_shear;
        });
        csv_out.add_function("inertia_foreaft",
                             [&reference_points, &idx_point]() { return reference_points[idx_point].inertia_foreaft; });
        csv_out.add_function("inertia_sideside", [&reference_points, &idx_point]() {
            return reference_points[idx_point].inertia_sideside;
        });
        for (idx_point = 0; idx_point < reference_points.size(); idx_point++) {
            csv_out.write_row();
        }

        // discretized points
        auto& discretized_points = turbine.elasto.tower->discretized_points;
        csv_out = seahowl::io::CustomCSV(
            (fs::path(logs_folder) / ("turbine" + std::to_string(idx_turbine + 1) + "_tower_points_discretized.csv"))
                .generic_string());
        csv_out.add_function("fraction",
                             [&discretized_points, &idx_point]() { return discretized_points[idx_point].fraction; });
        csv_out.add_function("density_linear",
                             [&discretized_points, &idx_point]() { return discretized_points[idx_point].density; });
        csv_out.add_function("stiffness_foreaft", [&discretized_points, &idx_point]() {
            return discretized_points[idx_point].stiffness_foreaft;
        });
        csv_out.add_function("stiffness_sideside", [&discretized_points, &idx_point]() {
            return discretized_points[idx_point].stiffness_sideside;
        });
        csv_out.add_function("stiffness_axial", [&discretized_points, &idx_point]() {
            return discretized_points[idx_point].stiffness_axial;
        });
        csv_out.add_function("stiffness_torsion", [&discretized_points, &idx_point]() {
            return discretized_points[idx_point].stiffness_torsion;
        });
        csv_out.add_function("stiffness_foreaft_shear", [&discretized_points, &idx_point]() {
            return discretized_points[idx_point].stiffness_foreaft_shear;
        });
        csv_out.add_function("stiffness_sideside_shear", [&discretized_points, &idx_point]() {
            return discretized_points[idx_point].stiffness_sideside_shear;
        });
        csv_out.add_function("inertia_foreaft", [&discretized_points, &idx_point]() {
            return discretized_points[idx_point].inertia_foreaft;
        });
        csv_out.add_function("inertia_sideside", [&discretized_points, &idx_point]() {
            return discretized_points[idx_point].inertia_sideside;
        });
        for (idx_point = 0; idx_point < discretized_points.size(); idx_point++) {
            csv_out.write_row();
        }
    }
}

seahowl::io::CustomCSV& OutputManager::create_new_csv(const std::string& csv_filename) {
    auto csv_filepath = (fs::path(output_folder) / csv_filename).generic_string();
    custom_csv_list.push_back(CustomCSV(csv_filepath));
    return custom_csv_list.back();
}
