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

/**
 * @brief Writes a CSV log of tower-like reference points (lineic density, stiffnesses, inertias).
 *
 * Used for both tower and monopile components, since they share the same TowerReferencePointElasto.
 *
 * @param[in] csv_filepath Output CSV file path.
 * @param[in] points Reference points (either reference_points or discretized_points of a TowerElasto).
 */
static void write_tower_points_csv(const std::string& csv_filepath,
                                   const std::vector<seahowl::elasto::TowerReferencePointElasto>& points) {
    size_t idx_point;
    auto csv_out = seahowl::io::CustomCSV(csv_filepath);
    csv_out.add_function("fraction", [&points, &idx_point]() { return points[idx_point].fraction; });
    csv_out.add_function("coordinates",
                         [&points, &idx_point]() -> seahowl::Vector3d { return points[idx_point].coordinates; });
    csv_out.add_function("density_linear", [&points, &idx_point]() { return points[idx_point].density; });
    csv_out.add_function("stiffness_foreaft", [&points, &idx_point]() { return points[idx_point].stiffness_foreaft; });
    csv_out.add_function("stiffness_sideside",
                         [&points, &idx_point]() { return points[idx_point].stiffness_sideside; });
    csv_out.add_function("stiffness_axial", [&points, &idx_point]() { return points[idx_point].stiffness_axial; });
    csv_out.add_function("stiffness_torsion", [&points, &idx_point]() { return points[idx_point].stiffness_torsion; });
    csv_out.add_function("stiffness_foreaft_shear",
                         [&points, &idx_point]() { return points[idx_point].stiffness_foreaft_shear; });
    csv_out.add_function("stiffness_sideside_shear",
                         [&points, &idx_point]() { return points[idx_point].stiffness_sideside_shear; });
    csv_out.add_function("inertia_foreaft", [&points, &idx_point]() { return points[idx_point].inertia_foreaft; });
    csv_out.add_function("inertia_sideside", [&points, &idx_point]() { return points[idx_point].inertia_sideside; });
    for (idx_point = 0; idx_point < points.size(); idx_point++) {
        csv_out.write_row();
    }
}

/**
 * @brief Writes a CSV log of blade reference points (offsets, twist, dampings, full 6x6 mass and stiffness matrices).
 *
 * The 6x6 mass and stiffness matrices are flattened in row-major order into 36 columns each,
 * named `mass_matrix_ij` / `stiffness_matrix_ij` with i,j in [0,5].
 *
 * @param[in] csv_filepath Output CSV file path.
 * @param[in] points Reference points (either reference_points or discretized_points of a BladeElasto).
 */
static void write_blade_points_csv(const std::string& csv_filepath,
                                   const std::vector<seahowl::elasto::BladeReferencePointElasto>& points) {
    size_t idx_point;
    auto csv_out = seahowl::io::CustomCSV(csv_filepath);
    csv_out.add_function("fraction", [&points, &idx_point]() { return points[idx_point].fraction; });
    csv_out.add_function("coordinates",
                         [&points, &idx_point]() -> seahowl::Vector3d { return points[idx_point].coordinates; });
    csv_out.add_function("offset_elastic_x", [&points, &idx_point]() { return points[idx_point].offset_elastic[0]; });
    csv_out.add_function("offset_elastic_y", [&points, &idx_point]() { return points[idx_point].offset_elastic[1]; });
    csv_out.add_function("offset_gravity_x", [&points, &idx_point]() { return points[idx_point].offset_gravity[0]; });
    csv_out.add_function("offset_gravity_y", [&points, &idx_point]() { return points[idx_point].offset_gravity[1]; });
    csv_out.add_function("structural_twist", [&points, &idx_point]() { return points[idx_point].structural_twist; });
    csv_out.add_function("damping_flapwise", [&points, &idx_point]() { return points[idx_point].damping_flapwise; });
    csv_out.add_function("damping_edgewise", [&points, &idx_point]() { return points[idx_point].damping_edgewise; });
    csv_out.add_function("damping_axial", [&points, &idx_point]() { return points[idx_point].damping_axial; });
    csv_out.add_function("damping_torsion", [&points, &idx_point]() { return points[idx_point].damping_torsion; });
    csv_out.add_function("damping_mass", [&points, &idx_point]() { return points[idx_point].damping_mass; });
    // 6x6 mass and stiffness matrices flattened in row-major order
    for (int ii = 0; ii < 6; ii++) {
        for (int jj = 0; jj < 6; jj++) {
            csv_out.add_function("mass_matrix_" + std::to_string(ii) + std::to_string(jj),
                                 [&points, &idx_point, ii, jj]() { return points[idx_point].mass_matrix(ii, jj); });
        }
    }
    for (int ii = 0; ii < 6; ii++) {
        for (int jj = 0; jj < 6; jj++) {
            csv_out.add_function(
                "stiffness_matrix_" + std::to_string(ii) + std::to_string(jj),
                [&points, &idx_point, ii, jj]() { return points[idx_point].stiffness_matrix(ii, jj); });
        }
    }
    for (idx_point = 0; idx_point < points.size(); idx_point++) {
        csv_out.write_row();
    }
}

/**
 * @brief Adds 9 columns named `prefix_ij` (i,j in [0,2]) for a 3x3 matrix returned by `getter()`.
 */
template <typename Getter>
static void add_inertia_columns(seahowl::io::CustomCSV& csv_out, const std::string& prefix, Getter getter) {
    for (int ii = 0; ii < 3; ii++) {
        for (int jj = 0; jj < 3; jj++) {
            csv_out.add_function(prefix + "_" + std::to_string(ii) + std::to_string(jj),
                                 [getter, ii, jj]() { return getter()(ii, jj); });
        }
    }
}

/**
 * @brief Writes a single-row CSV with mass / position / rpy / inertia of a body.
 *
 * @param[in] csv_filepath Output CSV file path.
 * @param[in] body Body to inspect.
 * @param[in] extra Optional callback to register extra columns (reference properties) before mass/state.
 */
static void write_body_csv(const std::string& csv_filepath,
                           seahowl::elasto::BodyElasto& body,
                           const std::function<void(seahowl::io::CustomCSV&)>& extra = {}) {
    auto csv_out = seahowl::io::CustomCSV(csv_filepath);
    if (extra) {
        extra(csv_out);
    }
    csv_out.add_function("mass", [&body]() { return body.get_mass(); });
    csv_out.add_function("position", [&body]() { return body.get_position(); });
    csv_out.add_function("rpy", [&body]() { return body.get_rpy_angles(); });
    add_inertia_columns(csv_out, "inertia", [&body]() { return body.get_inertia_matrix(); });
    csv_out.write_row();
}

void OutputManager::output_initial_logs() {
    std::string logs_folder = (fs::path(output_folder) / "logs").generic_string();
    spdlog::debug("Creating initial logs in {}.", logs_folder);
    fs::create_directories(logs_folder);
    // output
    for (int idx_turbine = 0; idx_turbine < system_core.turbines.size(); idx_turbine++) {
        auto& turbine = *system_core.turbines[idx_turbine];
        const std::string turbine_prefix = "turbine" + std::to_string(idx_turbine + 1);

        // tower
        write_tower_points_csv(
            (fs::path(logs_folder) / (turbine_prefix + "_tower_points_reference.csv")).generic_string(),
            turbine.elasto.tower->reference_points);
        write_tower_points_csv(
            (fs::path(logs_folder) / (turbine_prefix + "_tower_points_discretized.csv")).generic_string(),
            turbine.elasto.tower->discretized_points);

        // monopile foundation (if any)
        if (turbine.foundation) {
            try {
                auto& monopile = dynamic_cast<seahowl::core::Monopile&>(*turbine.foundation);
                auto& monopile_elasto = monopile.elasto;
                write_tower_points_csv(
                    (fs::path(logs_folder) / (turbine_prefix + "_monopile_points_reference.csv")).generic_string(),
                    monopile_elasto.reference_points);
                write_tower_points_csv(
                    (fs::path(logs_folder) / (turbine_prefix + "_monopile_points_discretized.csv")).generic_string(),
                    monopile_elasto.discretized_points);
            } catch (const std::bad_cast& e) {
                // do nothing if no monopile
            }
        }

        // blades
        for (size_t idx_blade = 0; idx_blade < turbine.rna.rotor.blades.size(); idx_blade++) {
            auto& blade_elasto = turbine.rna.rotor.blades[idx_blade]->elasto;
            const std::string blade_prefix = turbine_prefix + "_blade" + std::to_string(idx_blade + 1);
            write_blade_points_csv((fs::path(logs_folder) / (blade_prefix + "_points_reference.csv")).generic_string(),
                                   blade_elasto.reference_points);
            try {
                auto& blade_fea = dynamic_cast<seahowl::elasto::BladeElastoFEA&>(blade_elasto);
                write_blade_points_csv(
                    (fs::path(logs_folder) / (blade_prefix + "_points_discretized.csv")).generic_string(),
                    blade_fea.discretized_points);
            } catch (const std::bad_cast& e) {
                // no discretized points for non-FEA blades
            }
        }

        // RNA bodies (rotor/hub, shaft, nacelle): reference properties + body state
        auto& rna_elasto = turbine.rna.elasto;
        auto& rotor_elasto = *rna_elasto.rotor;
        const auto& hub_props = rotor_elasto.hub;
        const auto& shaft_props = rna_elasto.shaft;
        const auto& nacelle_props = rna_elasto.nacelle;

        // hub
        if (rotor_elasto.body_hub) {
            write_body_csv((fs::path(logs_folder) / (turbine_prefix + "_hub.csv")).generic_string(),
                           *rotor_elasto.body_hub, [&hub_props](seahowl::io::CustomCSV& csv_out) {
                               csv_out.add_function("ref_mass", [&hub_props]() { return hub_props.mass; });
                               csv_out.add_function("ref_overhang", [&hub_props]() { return hub_props.overhang; });
                               csv_out.add_function("ref_radius", [&hub_props]() { return hub_props.radius; });
                               csv_out.add_function("ref_position_from_apex",
                                                    [&hub_props]() { return hub_props.position_from_apex; });
                               add_inertia_columns(csv_out, "ref_inertia",
                                                   [&hub_props]() { return hub_props.inertia; });
                           });
        }

        // shaft
        if (rna_elasto.body_shaft) {
            write_body_csv((fs::path(logs_folder) / (turbine_prefix + "_shaft.csv")).generic_string(),
                           *rna_elasto.body_shaft, [&shaft_props](seahowl::io::CustomCSV& csv_out) {
                               csv_out.add_function("ref_tilt", [&shaft_props]() { return shaft_props.tilt; });
                               csv_out.add_function("ref_distance_from_towertop",
                                                    [&shaft_props]() { return shaft_props.distance_from_towertop; });
                           });
        }

        // nacelle
        if (rna_elasto.body_nacelle) {
            write_body_csv(
                (fs::path(logs_folder) / (turbine_prefix + "_nacelle.csv")).generic_string(), *rna_elasto.body_nacelle,
                [&nacelle_props](seahowl::io::CustomCSV& csv_out) {
                    csv_out.add_function("ref_mass", [&nacelle_props]() { return nacelle_props.mass; });
                    csv_out.add_function("ref_yaw_bearing_mass",
                                         [&nacelle_props]() { return nacelle_props.yaw_bearing_mass; });
                    csv_out.add_function("ref_position_from_towertop",
                                         [&nacelle_props]() { return nacelle_props.position_from_towertop; });
                    add_inertia_columns(csv_out, "ref_inertia", [&nacelle_props]() { return nacelle_props.inertia; });
                });
        }
    }
}

seahowl::io::CustomCSV& OutputManager::create_new_csv(const std::string& csv_filename) {
    auto csv_filepath = (fs::path(output_folder) / csv_filename).generic_string();
    custom_csv_list.push_back(CustomCSV(csv_filepath));
    return custom_csv_list.back();
}
