#include <seahowl/servo/controller_discon.h>

int main(int argc, char* argv[]) {
    std::cout << "Hello DISCON\n";
    seahowl::servo::DisconInterface discon_params;

    std::cout << "Initialize the controller\n";
    discon_params.ResetAll();
    discon_params.SetINFILE(u8"../data/IEA15MW/controller/DISCON.IN");

    std::cout << "Call the controller\n";
    discon_params.Call();
    discon_params.SetAvrSWAP(1, 1.0);

    std::cout << "Time  : " << discon_params.m_time << "\n";
    std::cout << "DT    : " << discon_params.m_dt << "\n";
    std::cout << "Pitch : " << discon_params.m_pitch << "\n";
    std::cout << "Torque: " << discon_params.m_torque << "\n";

    std::cout << "Call the controller" << std::endl;

    discon_params.m_time = 0.6f;

    discon_params.SetPitch(3.0f);
    discon_params.SetWindSpeed(13.0f);

    discon_params.Call();

    std::cout << "Time  : " << discon_params.m_time << "\n";
    std::cout << "DT    : " << discon_params.m_dt << "\n";
    std::cout << "Pitch : " << discon_params.m_pitch << "\n";
    std::cout << "Torque: " << discon_params.m_torque << "\n";

    auto increment = discon_params.GetAvrSWAP(94);

    discon_params.m_time = 1.5;

    // discon_params.SetPitch(3.0);
    discon_params.SetWindSpeed(7.0);

    discon_params.Call();

    std::cout << "Time  : " << discon_params.m_time << "\n";
    std::cout << "DT    : " << discon_params.m_dt << "\n";
    std::cout << "Pitch : " << discon_params.m_pitch << "\n";
    std::cout << "Torque: " << discon_params.m_torque << "\n";

    std::cout << "-------------------------------\n";
    discon_params.PrintAllOut(std::cout);

    /// std::cout << "MSG " << discon_params.avcMSG << std::endl;
    std::cout << "End controll\n";
    return 0;
}
