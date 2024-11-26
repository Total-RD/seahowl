#include "seahowl/io/write_csv.h"

#include "seahowl/core/system.h"
#include "seahowl/core/turbine.h"
#include "seahowl/core/blade.h"
#include "seahowl/elasto/blade_elasto.h"
#include "seahowl/elasto/tower_elasto.h"
#include "seahowl/elasto/rotor_elasto.h"
#include "seahowl/aero/blade_aero.h"
#include "seahowl/env/wind_models.h"
#include "seahowl/servo/controller.h"

#include <fstream>
#include <string>
#include <spdlog/spdlog.h>

using seahowl::PI;
using namespace seahowl::io;

CustomCSV::CustomCSV(const std::string& csv_filepath) : csv_filepath(csv_filepath) {}

void CustomCSV::add_function(const std::string& name, std::function<std::vector<double>()> function) {
    functions.push_back(std::pair<std::string, std::function<std::vector<double>()>>(name, function));
}

void CustomCSV::add_function(const std::string& name, std::function<seahowl::Vector3d()> function) {
    functions.push_back(
        std::pair<std::string, std::function<std::vector<double>()>>(name, [function]() -> std::vector<double> {
            auto vec = function();
            return {vec.x(), vec.y(), vec.z()};
        }));
}

void CustomCSV::add_function(const std::string& name, std::function<double()> function) {
    functions.push_back(std::pair<std::string, std::function<std::vector<double>()>>(
        name, [function]() -> std::vector<double> { return {function()}; }));
}

void CustomCSV::write_row() {
    std::string row_string = "";
    std::string header = "";  // only used if CSV is uninitialized
    for (auto const& function_pair : functions) {
        auto values = function_pair.second();
        size_t ivalue = 0;
        for (auto value : values) {
            row_string.append(std::to_string(value)).append(",");
            if (!is_initialized) {
                if (values.size() == 1) {
                    header.append(function_pair.first).append(",");
                } else if (values.size() == 3) {
                    std::string dim = "";
                    if (ivalue == 0) {
                        dim = "x";
                    } else if (ivalue == 1) {
                        dim = "y";
                    } else if (ivalue == 2) {
                        dim = "z";
                    }
                    size_t pos = function_pair.first.find("(");
                    if (pos != std::string::npos) {
                        pos += header.size();
                        header.append(function_pair.first).insert(pos, dim + " ").append(",");
                    } else {
                        header.append(function_pair.first).append(" " + dim).append(",");
                    }
                } else if (values.size() > 3) {
                    header.append(function_pair.first).append(" " + std::to_string(ivalue)).append(",");
                }
            }
            ivalue += 1;
        }
    }
    std::ofstream csv_file;
    if (!is_initialized) {
        if (csv_filepath.empty()) {
            throw std::runtime_error("Cannot write to CSV file as no filepath was provided.");
        }
        csv_file.open(csv_filepath);
        csv_file << header << "\n";
        is_initialized = true;
    } else {
        csv_file.open(csv_filepath, std::ios_base::app);
    }
    csv_file << row_string << "\n";
}

void CustomCSV::add_basic_turbine_info(const seahowl::core::Turbine& turbine, const seahowl::core::System& system) {
    add_function("time (s)", [&system]() { return system.get_time(); });
    add_function("wind (m/s)", [&system, &turbine]() {
        return system.fluid_model->get_fluid_velocity(turbine.rna.elasto.rotor->body_hub->get_position(),
                                                      system.get_time());
    });
    add_function("rpm", [&turbine]() { return turbine.rna.elasto.get_rpm(); });
    add_function("power (W)", [&turbine]() { return turbine.get_generated_power(); });
    add_function("pitch collective (rad)", [&turbine]() { return turbine.rna.elasto.rotor->pitch_collective; });
    add_function("torque elec (Nm)", [&turbine]() { return turbine.controller->get_torque_elec(); });
    add_function("axial thrust (N)", [&turbine]() { return turbine.rna.elasto.get_axial_thrust(); });
    add_function("axial torque (Nm)", [&turbine]() { return turbine.rna.elasto.get_axial_torque(); });
    add_function("rotor azimuth (rad)", [&turbine]() { return turbine.rna.elasto.get_azimuth(); });
    add_function("tower base moment", [&turbine]() { return turbine.tower.elasto.get_tower_base_moment(); });
    add_function("tower base force", [&turbine]() { return turbine.tower.elasto.get_tower_base_force(); });
    add_function("tower top moment", [&turbine]() { return turbine.tower.elasto.get_tower_top_moment(); });
    add_function("tower top force", [&turbine]() { return turbine.tower.elasto.get_tower_top_force(); });
    for (int idx_blade = 0; idx_blade < turbine.rna.blades.size(); idx_blade++) {
        auto& blade = *turbine.rna.blades[idx_blade];
        add_function("blade" + std::to_string(idx_blade + 1) + " wind (m/s)",
                     [&blade]() { return blade.aero.get_average_wind_velocity(); });
        add_function("blade" + std::to_string(idx_blade + 1) + " wind load (N)",
                     [&blade]() { return blade.aero.get_total_load(); });
        add_function("blade" + std::to_string(idx_blade + 1) + " root moment (Nm)",
                     [&blade]() { return blade.elasto.get_blade_root_moment(); });
        add_function("blade" + std::to_string(idx_blade + 1) + " azimuth (rad)", [&blade, &turbine]() {
            // check that blade_azimuth is between pi and -pi
            auto blade_azimuth = blade.elasto.azimuth0 + turbine.rna.elasto.get_azimuth();
            if (blade_azimuth < -PI || blade_azimuth > PI) {
                blade_azimuth = abs(std::fmod((blade_azimuth + 3 * PI), 2 * PI)) - PI;
            }
            return blade_azimuth;
        });
        add_function("blade" + std::to_string(idx_blade + 1) + " pitch (rad)",
                     [&blade]() { return blade.elasto.get_pitch(); });
    }
}
