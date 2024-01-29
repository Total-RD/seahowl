#include "seahowl/io/write_csv.h"

#include "seahowl/elasto/tower_elasto.h"
#include "seahowl/elasto/rotor_elasto.h"
#include "seahowl/elasto/blade_elasto.h"
#include "seahowl/aero/blade_aero.h"
#include "seahowl/env/wind_models.h"
#include "seahowl/core/blade.h"
#include "seahowl/core/system.h"
#include "seahowl/servo/controller.h"

#include <fstream>
#include <string>

using seahowl::PI;

void write_turbine_info_to_csv(std::string fileprefix, const seahowl::core::System& ssystem) {
    auto time = ssystem.get_time();
    for (int idx_turbine = 0; idx_turbine < ssystem.turbines.size(); idx_turbine++) {
        auto& turbine = *ssystem.turbines[idx_turbine];
        std::string filename;
        if (ssystem.turbines.size() == 1) {
            filename = fileprefix + ".csv";
        } else {
            filename = fileprefix + "_turbine" + std::to_string(idx_turbine) + ".csv";
        }
        std::ofstream myfile;
        if (time == 0.0) {
            myfile.open(filename);
            myfile << "time (s),wind x (m/s),wind y (m/s),wind z (m/s),rpm,power (W),pitch (rad),torque elec "
                      "(Nm),axial thrust (N),axial torque (Nm),rotor azimuth (rad),tower base moment x (Nm),tower base "
                      "moment y (Nm),tower base moment z (Nm),tower base force x (N),tower base force y (N),tower "
                      "base force z (N),tower top moment x (Nm),tower top moment y (Nm),tower top moment z (Nm),"
                      "tower top force x (N),tower top force y (N),tower top force z (N),";
            for (int idx_blade = 1; idx_blade < turbine.rna.blades.size() + 1; idx_blade++) {
                myfile << "blade" + std::to_string(idx_blade) + " wind x (m/s),";
                myfile << "blade" + std::to_string(idx_blade) + " wind y (m/s),";
                myfile << "blade" + std::to_string(idx_blade) + " wind z (m/s),";
                myfile << "blade" + std::to_string(idx_blade) + " load x (N),";
                myfile << "blade" + std::to_string(idx_blade) + " load y (N),";
                myfile << "blade" + std::to_string(idx_blade) + " load z (N),";
                myfile << "blade" + std::to_string(idx_blade) + " root moment x (Nm),";
                myfile << "blade" + std::to_string(idx_blade) + " root moment y (Nm),";
                myfile << "blade" + std::to_string(idx_blade) + " root moment z (Nm),";
                myfile << "blade" + std::to_string(idx_blade) + " azimuth (rad),";
            }
            myfile << "\n";
        } else {
            myfile.open(filename, std::ios_base::app);
        }
        myfile << std::to_string(time);
        myfile << ",";
        auto wind_velocity_hub =
            ssystem.fluid_model->get_fluid_velocity(turbine.rna.elasto.rotor->body_hub->get_position(), time);
        myfile << std::to_string(wind_velocity_hub.x());
        myfile << ",";
        myfile << std::to_string(wind_velocity_hub.y());
        myfile << ",";
        myfile << std::to_string(wind_velocity_hub.z());
        myfile << ",";
        myfile << std::to_string(turbine.rna.elasto.get_rpm());
        myfile << ",";
        myfile << std::to_string(turbine.get_generated_power());
        myfile << ",";
        myfile << std::to_string(turbine.rna.elasto.rotor->pitch_collective);
        myfile << ",";
        myfile << std::to_string(turbine.controller->get_torque_elec());
        myfile << ",";
        myfile << std::to_string(turbine.rna.elasto.get_axial_thrust());
        myfile << ",";
        myfile << std::to_string(turbine.rna.elasto.get_axial_torque());
        myfile << ",";
        myfile << std::to_string(turbine.rna.elasto.get_azimuth());
        myfile << ",";
        auto tower_base_moment = turbine.tower.elasto.get_tower_base_moment();
        myfile << std::to_string(tower_base_moment.x());
        myfile << ",";
        myfile << std::to_string(tower_base_moment.y());
        myfile << ",";
        myfile << std::to_string(tower_base_moment.z());
        myfile << ",";
        auto tower_base_force = turbine.tower.elasto.get_tower_base_force();
        myfile << std::to_string(tower_base_force.x());
        myfile << ",";
        myfile << std::to_string(tower_base_force.y());
        myfile << ",";
        myfile << std::to_string(tower_base_force.z());
        myfile << ",";
        auto tower_top_moment = turbine.tower.elasto.get_tower_top_moment();
        myfile << std::to_string(tower_top_moment.x());
        myfile << ",";
        myfile << std::to_string(tower_top_moment.y());
        myfile << ",";
        myfile << std::to_string(tower_top_moment.z());
        myfile << ",";
        auto tower_top_force = turbine.tower.elasto.get_tower_top_force();
        myfile << std::to_string(tower_top_force.x());
        myfile << ",";
        myfile << std::to_string(tower_top_force.y());
        myfile << ",";
        myfile << std::to_string(tower_top_force.z());
        myfile << ",";
        for (int ii = 0; ii < turbine.rna.blades.size(); ii++) {
            auto& blade = turbine.rna.blades[ii];
            auto wind_velocity_blade = blade->aero.get_average_wind_velocity();
            myfile << std::to_string(wind_velocity_blade.x());
            myfile << ",";
            myfile << std::to_string(wind_velocity_blade.y());
            myfile << ",";
            myfile << std::to_string(wind_velocity_blade.z());
            myfile << ",";
            auto load_blade = blade->aero.get_total_load();
            myfile << std::to_string(load_blade.x());
            myfile << ",";
            myfile << std::to_string(load_blade.y());
            myfile << ",";
            myfile << std::to_string(load_blade.z());
            myfile << ",";
            auto blade_root_moment = blade->elasto.get_blade_root_moment();
            myfile << std::to_string(blade_root_moment.x());
            myfile << ",";
            myfile << std::to_string(blade_root_moment.y());
            myfile << ",";
            myfile << std::to_string(blade_root_moment.z());
            myfile << ",";
            auto blade_azimuth = blade->elasto.azimuth0 + turbine.rna.elasto.get_azimuth();
            // check that blade_azimuth is between pi and -pi
            if (blade_azimuth < -PI || blade_azimuth > PI) {
                blade_azimuth = abs(std::fmod((blade_azimuth + 3 * PI), 2 * PI)) - PI;
            }
            myfile << std::to_string(blade_azimuth);
            myfile << ",";
        }
        myfile << "\n";
        myfile.close();
    }
}
