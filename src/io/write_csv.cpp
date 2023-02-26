#include <seahowl/io/write_csv.h>
#include <seahowl/elasto/blade_elasto.h>

#include <iostream>
#include <fstream>
#include <string>

void write_turbine_info_to_csv(std::string filename, const seahowl::core::System& ssystem, double time) {
    std::ofstream myfile;
    if (time == 0.0) {
        myfile.open(filename);
        myfile << "time (s),wind x (m/s),wind y (m/s),wind z (m/s),rpm,power (W),pitch (rad),torque elec (Nm),axial "
                  "thrust (N),axial torque (Nm),rotor azimuth (rad),";
        for (int ii = 1; ii < ssystem.turbine.blades.size() + 1; ii++) {
            myfile << "blade" + std::to_string(ii) + " wind x (m/s),";
            myfile << "blade" + std::to_string(ii) + " wind y (m/s),";
            myfile << "blade" + std::to_string(ii) + " wind z (m/s),";
            myfile << "blade" + std::to_string(ii) + " load x (N),";
            myfile << "blade" + std::to_string(ii) + " load y (N),";
            myfile << "blade" + std::to_string(ii) + " load z (N),";
            myfile << "blade" + std::to_string(ii) + " azimuth (rad),";
        }
        myfile << "\n";
    } else {
        myfile.open(filename, std::ios_base::app);
    }
    myfile << std::to_string(time);
    myfile << ",";
    auto wind_velocity_hub =
        ssystem.wind_model->get_wind_velocity(ssystem.turbine.rotor.elasto.body_hub->GetPos(), time);
    myfile << std::to_string(wind_velocity_hub.x());
    myfile << ",";
    myfile << std::to_string(wind_velocity_hub.y());
    myfile << ",";
    myfile << std::to_string(wind_velocity_hub.z());
    myfile << ",";
    myfile << std::to_string(ssystem.turbine.rotor.elasto.get_rpm());
    myfile << ",";
    myfile << std::to_string(ssystem.turbine.get_generated_power());
    myfile << ",";
    myfile << std::to_string(ssystem.turbine.rotor.elasto.pitch_collective);
    myfile << ",";
    myfile << std::to_string(ssystem.turbine.controller->get_torque_elec());
    myfile << ",";
    myfile << std::to_string(ssystem.turbine.rotor.elasto.get_axial_thrust());
    myfile << ",";
    myfile << std::to_string(ssystem.turbine.rotor.elasto.get_axial_torque());
    myfile << ",";
    myfile << std::to_string(ssystem.turbine.rotor.elasto.get_azimuth());
    myfile << ",";
    for (int ii = 0; ii < ssystem.turbine.blades.size(); ii++) {
        auto& blade = ssystem.turbine.blades[ii];
        auto wind_velocity_blade = blade->aero->get_average_wind_velocity();
        myfile << std::to_string(wind_velocity_blade.x());
        myfile << ",";
        myfile << std::to_string(wind_velocity_blade.y());
        myfile << ",";
        myfile << std::to_string(wind_velocity_blade.z());
        myfile << ",";
        auto load_blade = blade->aero->get_total_load();
        myfile << std::to_string(load_blade.x());
        myfile << ",";
        myfile << std::to_string(load_blade.y());
        myfile << ",";
        myfile << std::to_string(load_blade.z());
        myfile << ",";
        auto blade_azimuth = blade->elasto->azimuth0 + ssystem.turbine.rotor.elasto.get_azimuth();
        // check that blade_azimuth is between pi and -pi
        if (blade_azimuth < -chrono::CH_C_PI || blade_azimuth > chrono::CH_C_PI) {
            blade_azimuth =
                abs(std::fmod((blade_azimuth + 3 * chrono::CH_C_PI), 2 * chrono::CH_C_PI)) - chrono::CH_C_PI;
        }
        myfile << std::to_string(blade_azimuth);
        myfile << ",";
    }
    myfile << "\n";
    myfile.close();
}
