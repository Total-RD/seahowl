#include "seahowl/servo/controller_discon.h"

#include <stdexcept>
#include <vector>
#include <string>
#include <iostream>
#include <filesystem>
#ifdef __unix__
    #include <dlfcn.h>
#endif
#ifdef _WIN32
    #include <windows.h>
#endif

seahowl::servo::ControllerDISCON::ControllerDISCON(std::string infile, std::string libfile_in, std::string outname) {
    has_pitch_control = true;
    has_torque_control = true;

    if (!std::filesystem::exists(std::filesystem::path(libfile_in))) {
        throw std::runtime_error("Dynamic library path for DISCON routine does not exist: " + libfile_in + ".");
    }

    libfile = libfile_in;
    pImpl.ResetAll();
    pImpl.SetINFILE(infile);
    pImpl.SetOUTNAME(outname);
}

void seahowl::servo::ControllerDISCON::step(double time, double dt, const seahowl::core::Turbine& turbine) {
    auto omega_rotor = turbine.rotor.elasto.get_rpm() * (2 * chrono::CH_C_PI / 60.0);
    auto omega_generator = turbine.get_generator_rpm() * (2 * chrono::CH_C_PI / 60.0);
    auto pitch_collective = turbine.rotor.elasto.pitch_collective;
    auto rotor_azimuth = turbine.rotor.elasto.get_azimuth();
    auto power = turbine.get_generated_power();
    this->step(time, dt, omega_rotor, omega_generator, pitch_collective, rotor_azimuth, power);
}

void seahowl::servo::ControllerDISCON::step(double time,
                                            double dt,
                                            double omega_rotor,
                                            double omega_generator,
                                            double pitch_collective,
                                            double rotor_azimuth,
                                            double power) {
    // update variables
    update_turbine_variables(time, dt, omega_rotor, omega_generator, pitch_collective, rotor_azimuth, power);

    // call controller
    pImpl.Call();
}

void seahowl::servo::ControllerDISCON::update_turbine_variables(double time,
                                                                double dt,
                                                                double omega_rotor,
                                                                double omega_generator,
                                                                double pitch_collective,
                                                                double rotor_azimuth,
                                                                double power) {
    // rotor speed
    pImpl.SetRotorSpeed(omega_rotor);
    // generator speed
    pImpl.SetGeneratorSpeed(omega_generator);

    // time
    pImpl.SetTime(time);
    pImpl.SetDeltaTime(dt);

    // collective pitch
    pImpl.SetPitch(pitch_collective);

    // azimuth
    // auto rotor_azimuth = pImpl.GetAvrSWAP(60) + omega * dt;
    // if (rotor_azimuth > 2 * 3.14) {
    //    rotor_azimuth -= 2 * 3.14;
    //}
    pImpl.SetRotorAzimuth(rotor_azimuth);

    // power
    pImpl.SetGeneratedPower(power);
}

void seahowl::servo::ControllerDISCON::init(double time, double dt, const seahowl::core::Turbine& turbine) {
    auto omega_rotor = turbine.rotor.elasto.get_rpm() * (2 * chrono::CH_C_PI / 60.0);
    auto omega_generator = turbine.get_generator_rpm() * (2 * chrono::CH_C_PI / 60.0);
    auto pitch_collective = turbine.rotor.elasto.pitch_collective;
    auto rotor_azimuth = turbine.rotor.elasto.get_azimuth();
    auto nblades = turbine.blades.size();
    this->init(time, dt, omega_rotor, omega_generator, pitch_collective, rotor_azimuth, nblades);
}

void seahowl::servo::ControllerDISCON::init(double time,
                                            double dt,
                                            double omega_rotor,
                                            double omega_generator,
                                            double pitch_collective,
                                            double rotor_azimuth,
                                            size_t nblades) {
    pImpl.SetNumberOfBlades(nblades);

    // update variables
    update_turbine_variables(time, dt, omega_rotor, omega_generator, pitch_collective, rotor_azimuth, 0.0);

    pImpl.SetAvrSWAP(27, 10.0);  // estimated wind speed (needs to be != 0 at init for it to work in ROSCO!)

    pImpl.Init(libfile);
}

double seahowl::servo::ControllerDISCON::get_torque_elec() const {
    double torque_elec = pImpl.GetAvrSWAP(47);
    return torque_elec;
}

double seahowl::servo::ControllerDISCON::get_collective_pitch() const {
    double collective_pitch = pImpl.GetAvrSWAP(45);
    return collective_pitch;
}

/**@brief Discon controller */
namespace discon {
struct ParamDef;
}

///@brief DISCON Parameter
struct discon::ParamDef {
    /**@brief Fortran index record number*/
    size_t index;
    /**@brief IN|OUT|INOUT status*/
    std::string inout;

    /**@brief type  R|I[C|L*/
    char type;

    /**@brief Description */
    std::string description;

    /**@brief Unit */
    std::string unit;
};

namespace discon {

/// <summary>
/// ArrayInfo for DISCON
/// from Bladed user-defined controllers in versions prior to version 4.4
/// </summary>
static std::vector<discon::ParamDef> ArrayInfo{

    ///@brief Dummy array (in order to start ids from 1)
    {0, "both", 'I', "Dummy array", "-"},
    {1, "in", 'I', "See Section A.2", "-"},
    {2, "in", 'R', "Current time", "s"},
    {3, "in", 'R', "Communication interval", "s"},
    {4, "in", 'R', "Blade 1 pitch angle", "rad"},
    {5, "in", 'R', "Below-rated pitch angle set-point 1", "rad"},
    {6, "in", 'R', "Minimum pitch angle 1", "rad"},
    {7, "in", 'R', "Maximum pitch angle 1", "rad"},
    {8, "in", 'R', "Minimum pitch rate (most negative value allowed)", "rad/s"},
    {9, "in", 'R', "Maximum pitch rate", "rad/s"},
    {10, "in", 'I', "0 = pitch position actuator, 1 = pitch rate actuator", "-"},
    {11, "in", 'R', "Current demanded pitch angle", "rad"},
    {12, "in", 'R', "Current demanded pitch rate", "rad/s"},
    {13, "in", 'R', "Demanded power", "W"},
    {14, "in", 'R', "Measured shaft power", "W"},
    {15, "in", 'R', "Measured electrical power output", "W"},
    {16, "in", 'R', "Optimal mode gain", "Nm/(rad/s) 2"},
    {17, "in", 'R', "Minimum generator speed", "rad/s"},
    {18, "in", 'R', "Optimal mode maximum speed", "rad/s"},
    {19, "in", 'R', "Demanded generator speed above rated", "rad/s"},
    {20, "in", 'R', "Measured generator speed", "rad/s"},
    {21, "in", 'R', "Measured rotor speed", "rad/s"},
    {22, "in", 'R', "Demanded generator torque above rated", "Nm"},
    {23, "in", 'R', "Measured generator torque", "Nm"},
    {24, "in", 'R', "Measured yaw error", "rad"},
    {25, "in", 'I', "Start of below-rated torque-speed look-up table =R", "Record no."},
    {26, "in", 'I', "No. of points in torque-speed look-up table =N", "-"},
    {27, "in", 'R', "Hub wind speed", "m/s"},
    {28, "in", 'I', "Pitch control: 0 = collective, 1 = individual", "-"},
    {29, "in", 'I', "Yaw control: 0 = yaw rate control, 1 = yaw torque control", "-"},
    {30, "in", 'R', "Blade 1 root out of plane bending moment", "Nm"},
    {31, "in", 'R', "Blade 2 root out of plane bending moment", "Nm"},
    {32, "in", 'R', "Blade 3 root out of plane bending moment", "Nm"},
    {33, "in", 'R', "Blade 2 pitch angle", "rad"},
    {34, "in", 'R', "Blade 3 pitch angle", "rad"},
    {35, "both", 'I', "Generator contactor", "-"},
    {36, "both", 'I', "Shaft brake status: 0=off, 1=Brake 1 on", "-"},
    {37, "in", 'R', "Nacelle angle from North", "rad"},
    {38, "out", 'R', "Reserved", "-"},
    {39, "out", 'R', "Reserved", "-"},
    {40, "out", 'R', "Reserved", "-"},
    {41, "out", 'R', "Demanded yaw actuator torque", "Nm"},
    {42, "out", 'R', "Demanded blade 1 individual pitch position or rate", "rad or rad/s"},
    {43, "out", 'R', "Demanded blade 2 individual pitch position or rate", "rad or rad/s"},
    {44, "out", 'R', "Demanded blade 3 individual pitch position or rate4", "rad or rad/s"},
    {45, "out", 'R', "Demanded pitch angle (Collective pitch)", "rad"},
    {46, "out", 'R', "Demanded pitch rate (Collective pitch)", "rad/s"},
    {47, "out", 'R', "Demanded generator torque", "Nm"},
    {48, "out", 'R', "Demanded nacelle yaw rate", "rad/s"},
    // Bug in bladed doc ? 2 times in the doc a in or out . 2 different significations {49, "out", 'I', "Message
    // length
    // OR -M0", "-"},
    {49, "inout", 'I', "Maximum no. of characters allowed in the MESSAGE", "-"},
    {50, "in", 'I', "No. of characters in the 'INFILE' argument", "-"},
    {51, "in", 'I', "No. of characters in the 'OUTNAME' argument", "-"},
    {52, "in", 'I', "DLL interface version number (reserved for futureuse)", "-"},
    {53, "in", 'R', "Tower top fore-aft acceleration", "m/s2"},
    {54, "in", 'R', "Tower top side to side acceleration", "m/s2"},
    {55, "out", 'I', "Pitch override", "-"},
    {56, "out", 'I', "Torque override", "-"},
    {57, "out", 'R', "Reserved", ""},
    {58, "out", 'R', "Reserved", ""},
    {59, "out", 'R', "Reserved", ""},
    {60, "both", 'R', "Rotor azimuth angle", "rad"},
    {61, "in", 'I', "No. of blades", "-"},
    {62, "in", 'I', "Max. number of values which can be returned for logging", "-"},
    {63, "in", 'I', "Record number for start of logging output", "-"},
    {64, "in", 'I', "Max. no. of characters which can be returned in 'OUTNAME'", "-"},
    {65, "out", 'I', "Number of variables returned for logging", "-"},
    {66, "in", 'R', "Reserved", ""},
    {67, "in", 'R', "Reserved", ""},
    {68, "in", 'R', "Reserved", ""},
    {69, "in", 'R', "Blade 1 root in plane bending moment", "Nm"},
    {70, "in", 'R', "Blade 2 root in plane bending moment", "Nm"},
    {71, "in", 'R', "Blade 3 root in plane bending moment", "Nm"},
    {72, "out", 'R', "Generator start-up resistance", "ohm/phase"},
    {73, "in", 'R', "Rotating hub My (GL co-ords)", "Nm"},
    {74, "in", 'R', "Rotating hub Mz (GL co-ords)", "Nm"},
    {75, "in", 'R', "Fixed hub My (GL co-ords)", "Nm"},
    {76, "in", 'R', "Fixed hub Mz (GL co-ords)", "Nm"},
    {77, "in", 'R', "Yaw bearing My (GL co-ords)", "Nm"},
    {78, "in", 'R', "Yaw bearing Mz (GL co-ords)", "Nm"},
    {79, "out", 'I', "Request for loads", "-"},
    {80, "out", 'I', "1 = Variable slip current demand at position 81", "-"},
    {81, "both", 'R', "Variable slip current demand", "A"},
    {82, "in", 'R', "Nacelle roll acceleration", "rad/s2"},
    {83, "in", 'R', "Nacelle nodding acceleration", "rad/s"},
    {84, "in", 'R', "Nacelle yaw acceleration", "rad/s2"},
    {85, "", 'R', "Reserved", "-"},
    {86, "", 'R', "Reserved", "-"},
    {87, "", 'R', "Reserved", "-"},
    {88, "", 'R', "Reserved", "-"},
    {89, "", 'R', "Reserved", "-"},
    {90, "in", 'R', "Real time simulation time step", "s"},
    {91, "in", 'R', "Real time simulation time step multiplier", "-"},
    {92, "out", 'R', "Mean wind speed increment", "m/s"},
    {93, "out", 'R', "Turbulence intensity increment", "%"},
    {94, "out", 'R', "Wind direction increment", "rad"},
    {95, "", 'R', "Reserved", "-"},
    {96, "", 'R', "Reserved", "-"},
    {97, "in", 'I', "Safety system number that has been activated", "-"},
    {98, "out", 'I', "Safety system number to activate", "-"},
    {99, "in", 'I', "Reserved", ""},
    {100, "in", 'I', "Reserved", ""},
    {101, "in", 'R', "Reserved", ""},
    {102, "out", 'I', "Yaw control flag", "-"},
    {103, "out", 'R', "Yaw stiffness if record 102 = 1 or 3", "-"},
    {104, "out", 'R', "Yaw damping if record 102 = 2 or 3", "-"},
    {105, "in", 'R', "Reserved", ""},
    {106, "in", 'R', "Reserved", ""},
    {107, "out", 'R', "Brake torque demand", "Nm"},
    {108, "out", 'R', "Yaw brake torque demand", "Nm"},
    {109, "in", 'R', "Shaft torque (= hub Mx for clockwise rotor)", "Nm"},
    {110, "in", 'R', "Hub Fixed Fx", "N"},
    {111, "in", 'R', "Hub Fixed Fy", "N"},
    {112, "in", 'R', "Hub Fixed Fz", "N"},
    {113, "in", 'R', "Network voltage disturbance factor", "-"},
    {114, "in", 'R', "Network frequency disturbance factor", "-"},
    {115, "", 'R', "Reserved", ""},
    {116, "", 'R', "Reserved", ""},
    {117, "in", 'I', "Controller state", "-"},
    {118, "in", 'R', "Settling time (time to start writing output)", "s"},
    {119, "", 'R', "Reserved", ""},
    {120, "both", 'R', "User-defined variables 1 to 10", ""},
    {121, "both", 'R', "User-defined variables 1 to 10", ""},
    {122, "both", 'R', "User-defined variables 1 to 10", ""},
    {123, "both", 'R', "User-defined variables 1 to 10", ""},
    {124, "both", 'R', "User-defined variables 1 to 10", ""},
    {125, "both", 'R', "User-defined variables 1 to 10", ""},
    {126, "both", 'R', "User-defined variables 1 to 10", ""},
    {127, "both", 'R', "User-defined variables 1 to 10", ""},
    {128, "both", 'R', "User-defined variables 1 to 10", ""},
    {129, "both", 'R', "User-defined variables 1 to 10", ""},
    {130, "", 'R', "Reserved", ""},
    {131, "", 'R', "Reserved", ""},
    {132, "", 'R', "Reserved", ""},
    {133, "", 'R', "Reserved", ""},
    {134, "", 'R', "Reserved", ""},
    {135, "", 'R', "Reserved", ""},
    {136, "", 'R', "Reserved", ""},
    {137, "", 'R', "Reserved", ""},
    {138, "", 'R', "Reserved", ""},
    {139, "", 'R', "Reserved", ""},
    {140, "", 'R', "Reserved", ""},
    {141, "", 'R', "Reserved", ""},
    {142, "", 'R', "Reserved", ""},
    {143, "in", 'R', "Teeter angle", "rad"},
    {144, "in", 'R', "Teeter velocity", "rad/s"},
    {145, "", 'R', "Reserved", ""},
    {146, "", 'R', "Reserved", ""},
    {147, "", 'R', "Reserved", ""},
    {148, "", 'R', "Reserved", ""},
    {149, "", 'R', "Reserved", ""},
    {150, "", 'R', "Reserved", ""},
    {151, "", 'R', "Reserved", ""},
    {152, "", 'R', "Reserved", ""},
    {153, "", 'R', "Reserved", ""},
    {154, "", 'R', "Reserved", ""},
    {155, "", 'R', "Reserved", ""},
    {156, "", 'R', "Reserved", ""},
    {157, "", 'R', "Reserved", ""},
    {158, "", 'R', "Reserved", ""},
    {159, "", 'R', "Reserved", ""},
    {160, "", 'R', "Reserved", ""},
    {161, "in", 'I', "Controller failure flag", "-"},
    {162, "in", 'R', "Yaw bearing angular position", "rad"},
    {163, "in", 'R', "Yaw bearing angular velocity", "rad/s"},
    {164, "in", 'R', "Yaw bearing angular acceleration", "rad/s2"}};
}  // namespace discon

/*

Record
number
Data
flow8
Data
type
9
Description
See
note(
s)
Units
1 in I See Section A.2 -




*/

/*

CHARACTER(KIND=C_CHAR),         INTENT(IN   )   :: accINFILE(NINT(avrSWAP(50)))     ! The name of the parameter
input file CHARACTER(KIND=C_CHAR),         INTENT(IN   )   :: avcOUTNAME(NINT(avrSWAP(51)))    ! OUTNAME (Simulation
RootName) CHARACTER(KIND=C_CHAR),         INTENT(INOUT)   :: avcMSG(NINT(avrSWAP(49)))        ! MESSAGE (Message
from DLL to simulation code [ErrMsg])  The message which will be displayed by the calling program if aviFAIL <> 0.
CHARACTER(SIZE(avcOUTNAME)-1)                   :: RootName                         ! a Fortran version of the input
C string (not considered an array here)    [subtract 1 for the C null-character] CHARACTER(SIZE(avcMSG)-1) :: ErrMsg


    LocalVar%GenSpeed           = avrSWAP(20)
    LocalVar%RotSpeed           = avrSWAP(21)
    LocalVar%GenTqMeas          = avrSWAP(23)
    LocalVar%Y_M                = avrSWAP(24)
    LocalVar%HorWindV           = avrSWAP(27)
    LocalVar%rootMOOP(1)        = avrSWAP(30)
    LocalVar%rootMOOP(2)        = avrSWAP(31)
    LocalVar%rootMOOP(3)        = avrSWAP(32)
    LocalVar%FA_Acc             = avrSWAP(53)
    LocalVar%NacIMU_FA_Acc      = avrSWAP(83)
    LocalVar%Azimuth            = avrSWAP(60)
    LocalVar%NumBl              = NINT(avrSWAP(61))

    ! --- NJA: usually feedback back the previous pitch command helps for numerical stability, sometimes it does
not... IF (LocalVar%iStatus == 0) THEN LocalVar%BlPitch(1) = avrSWAP(4) LocalVar%BlPitch(2) = avrSWAP(33)
        LocalVar%BlPitch(3) = avrSWAP(34)
    ELSE
        LocalVar%BlPitch(1) = LocalVar%PitCom(1)
        LocalVar%BlPitch(2) = LocalVar%PitCom(2)
        LocalVar%BlPitch(3) = LocalVar%PitCom(3)
    ENDIF


    ! Set unused outputs to zero (See Appendix A of Bladed User's Guide):
    avrSWAP(35) = 1.0 ! Generator contactor status: 1=main (high speed) variable-speed generator
    avrSWAP(36) = 0.0 ! Shaft brake status: 0=off
    avrSWAP(41) = 0.0 ! Demanded yaw actuator torque
    avrSWAP(46) = 0.0 ! Demanded pitch rate (Collective pitch)
    avrSWAP(55) = 0.0 ! Pitch override: 0=yes
    avrSWAP(56) = 0.0 ! Torque override: 0=yes
    avrSWAP(65) = 0.0 ! Number of variables returned for logging
    avrSWAP(72) = 0.0 ! Generator start-up resistance
    avrSWAP(79) = 0.0 ! Request for loads: 0=none
    avrSWAP(80) = 0.0 ! Variable slip current status
    avrSWAP(81) = 0.0 ! Variable slip current demand


    */

/*


auto* handler = dlopen("/home/tridelat/work/NREL/ROSCO/ROSCO/build/libdiscon.so");
auto discon1 = dlsym(handler, "DISCON");
*/

void seahowl::servo::DisconController::Init(std::string libfile) {
    // Load dynamic library and point to DISCON routine
#ifdef __unix__
    void* handler = dlopen(libfile.c_str(), RTLD_LAZY);
    DISCON = (DISCON_routine)dlsym(handler, "DISCON");
#endif
#ifdef _WIN32
    HMODULE handler = LoadLibrary(libfile.c_str());
    DISCON = (DISCON_routine)GetProcAddress(handler, "DISCON");
#endif

    avrSWAP[58] = 500;  // Buffer chaar size
    avrSWAP[50] = 500;  // self.char_buffer
    avrSWAP[51] = 500;  // self.char_buffer

    aviFAIL = 1;

    // First step
    ResetFirst();
    Call();
    SetAvrSWAP(1, 1.0);  // iStatus : standard  step (not the first, which was already just called)
};

void seahowl::servo::DisconController::ResetAll() {
    for (auto& v : avrSWAP) {
        v = 0.0;
    }
}

void seahowl::servo::DisconController::PrintAllOut(std::ostream& ssout) const {
    for (int i = 1; i < 150; ++i) {
        auto& ap = discon::ArrayInfo.at(i);
        ///@todo Better to use GetAvrSWAP but without log
        if (ap.inout == "out" || ap.inout == "both") {
            ssout << "value[" << ap.index << "]=" << avrSWAP[ap.index - 1] << " " << ap.unit << "; " << ap.description
                  << "\n";
        }
    }
}

void seahowl::servo::DisconController::SetINFILE(std::string name) {
    std::cout << "Set INFILE: '" << name << "'\n";
    strcpy(accINFILE, name.c_str());
    SetAvrSWAP(50, name.length());
}

void seahowl::servo::DisconController::SetOUTNAME(std::string name) {
    std::cout << "Set OUTNAME:'" << name << "'\n";
    strcpy(avcOUTNAME, name.c_str());
    SetAvrSWAP(51, name.length());
}

void seahowl::servo::DisconController::SetPitch(double pitch_angle) {
    SetAvrSWAP(4, static_cast<float>(pitch_angle));
    SetAvrSWAP(33, static_cast<float>(pitch_angle));
    SetAvrSWAP(34, static_cast<float>(pitch_angle));
}

void seahowl::servo::DisconController::SetWindSpeed(double ws) {
    SetAvrSWAP(27, static_cast<float>(ws));
}

void seahowl::servo::DisconController::SetRotorSpeed(double omega) {
    SetAvrSWAP(21, omega);
}

void seahowl::servo::DisconController::SetGeneratorSpeed(double omega) {
    SetAvrSWAP(20, omega);
}

void seahowl::servo::DisconController::SetTime(double time) {
    SetAvrSWAP(2, time);
}

void seahowl::servo::DisconController::SetDeltaTime(double dt) {
    SetAvrSWAP(3, dt);
}

void seahowl::servo::DisconController::SetRotorAzimuth(double azimuth) {
    SetAvrSWAP(60, azimuth);
}

void seahowl::servo::DisconController::SetGeneratedPower(double power) {
    SetAvrSWAP(15, power);
}

void seahowl::servo::DisconController::SetNumberOfBlades(size_t nblades) {
    SetAvrSWAP(61, nblades);
}

void seahowl::servo::DisconController::SetAvrSWAP(size_t index, float value, bool log) {
    if (index < 1 || index > MAX_SWAP)
        throw std::runtime_error("avrSWAP index out of bounds");

    auto& ap = discon::ArrayInfo.at(index);
    if (ap.index != index) {
        std::cerr << "WARNING: mismatch array index " << index << " and record number " << ap.index << "\n";
    }
    if (ap.inout != "in" && ap.inout != "both") {
        std::cerr << "ERROR: avrSWAP[" << index << "] " << ap.inout << "not an input parameter!\n";
    }

    if (log)
        std::cout << "Set: " << ap.description << ": " << value << " " << ap.unit << "\n";

    avrSWAP[ap.index - 1] = value;
}

void seahowl::servo::DisconController::SetAvrSWAP(size_t index, size_t value, bool log) {
    SetAvrSWAP(index, static_cast<float>(value));
}
void seahowl::servo::DisconController::SetAvrSWAP(size_t index, double value, bool log) {
    SetAvrSWAP(index, static_cast<float>(value));
}

float seahowl::servo::DisconController::GetAvrSWAP(size_t index, bool log) const {
    if (index < 1 || index > MAX_SWAP)
        throw std::runtime_error("avrSWAP index out of bounds");

    auto& ap = discon::ArrayInfo.at(index);
    if (ap.index != index) {
        std::cerr << "WARNING: mismatch array index " << index << " and record number " << ap.index << "\n";
    }
    if (ap.inout != "out" && ap.inout != "both") {
        std::cerr << "ERROR: avrSWAP[" << index << "] " << ap.inout << "not an output parameter!\n";
    }

    auto& value = avrSWAP[index - 1];
    if (log)
        std::cout << "Get: " << ap.description << ": " << value << " " << ap.unit << "\n";

    return value;
}

void seahowl::servo::DisconController::Call() {
    DISCON(avrSWAP, &aviFAIL, accINFILE, avcOUTNAME, avcMSG);
}
