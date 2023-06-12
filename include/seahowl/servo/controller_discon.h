#pragma once

#include <iostream>
#include <cstring>

#include <seahowl/servo/controller.h>

/// <summary>
/// Fortran Fonction definition of DISCO (ROSCO) controller
/// </summary>
/// @todo shoudl be private
/// <param name="avrSWAP"></param>
/// <param name="aviFAIL"></param>
/// <param name="accINFILE"></param>
/// <param name="avcOUTNAME"></param>
/// <param name="avcMSG"></param>

namespace seahowl {
namespace servo {

/**@brief ROSCO Discon wrapping (adapter) interface
 *
 * @todo Set as Pimpl private implementation of seahowl::servo::Controller class
 */
struct DisconController {
    // declare DISCON routine type and variable to load from dynamic library
    typedef void (*DISCON_routine)(float* avrSWAP, int* aviFAIL, char* accINFILE, char* avcOUTNAME, char* avcMSG);
    DISCON_routine DISCON;

    float& m_time = avrSWAP[1];     ///<@brief Time
    float& m_dt = avrSWAP[2];       ///<@brief Time step
    float& m_pitch = avrSWAP[41];   ///<@brief Pitch return controller states
    float& m_torque = avrSWAP[46];  ///<@brief Torque return controller states

    /// <summary>
    /// Reset Controller state as initial
    ///
    /// </summary>
    void ResetFirst() {
        avrSWAP[0] = 0;  // Initial step iStatus
    }

    /// <summary>
    /// Reset all controller fields to 0
    ///
    /// </summary>
    void ResetAll();

    /// <summary>
    /// Initialize the controller parameters
    /// </summary>
    /// <param name="dt">timestep</param>
    /// <param name="omega">rotor speed</param>
    /// <param name="pitch">pitch collective</param>
    /// <param name="nblades">number of blades</param>
    void Init(std::string libfile);

    /// <summary>
    /// Call the DISCON controller
    /// </summary>
    void Call();

    /// <summary>
    /// Helper to set the guess pitch
    /// </summary>
    /// <param name="pitch_angle"></param>
    void SetPitch(double pitch_angle);

    void SetPitchBlade(int index_blade, double pitch_angle);

    /// <summary>
    /// Helper to set Inflow wind speed
    /// </summary>
    /// <param name="ws">The inflow wind speed. m.s^{-1}</param>
    void SetWindSpeed(double ws);

    /// <summary>
    /// Helper to set rotor speed
    /// </summary>
    /// <param name="omega">The rotor speed. rad.s^{-1}</param>
    void SetRotorSpeed(double omega);

    /// <summary>
    /// Helper to set rotor speed
    /// </summary>
    /// <param name="omega">The rotor speed. rad.s^{-1}</param>
    void SetGeneratorSpeed(double omega);

    /// <summary>
    /// Helper to set time
    /// </summary>
    /// <param name="time">The time. s</param>
    void SetTime(double time);

    /// <summary>
    /// Helper to set time step size
    /// </summary>
    /// <param name="dt">The time step size. s</param>
    void SetDeltaTime(double dt);

    /// <summary>
    /// Helper to set rotor azimuth
    /// </summary>
    /// <param name="azimuth">The rotor azimuth. rad</param>
    void SetRotorAzimuth(double azimuth);

    /// <summary>
    /// Helper to set the generated power
    /// </summary>
    /// <param name="power">The generated power. W</param>
    void SetGeneratedPower(double power);

    /// <summary>
    /// Helper to set number of blades
    /// </summary>
    /// <param name="nblades">The number of blades.</param>
    void SetNumberOfBlades(size_t nblades);

    /// <summary>
    /// Set Value in avrSWAP array of DISCON
    /// </summary>
    /// <param name="index">Index Fortran. (eg +1 compared to C)</param>
    /// <param name="value">The value to set</param>
    /// <param name="log">If true print the value on standard output</param>
    void SetAvrSWAP(size_t index, float value, bool log = false);

    // Helper to force the cast of value to float
    void SetAvrSWAP(size_t index, size_t value, bool log = false);
    // Helper to force the cast of value to float
    void SetAvrSWAP(size_t index, double value, bool log = false);

    /// <summary>
    /// Get Value from avrSWAP array of DISCON
    /// </summary>
    /// <param name="index">Index Fortran. (eg +1 compared to C)</param>
    /// <param name="log">If true print the value on standard output</param>
    float GetAvrSWAP(size_t index, bool log = false) const;

    /// <summary>
    /// Set input filename with path relative to working directory.
    /// Path is used for other files
    /// ex! control/DISCON.in find other files in control directory
    /// </summary>
    /// <param name="name">DISCON.IN input file path</param>
    void SetINFILE(std::string name = u8"DISCON.IN");

    /// <summary>
    /// Set output base name (relative to working directory).
    /// </summary>
    /// <param name="name">a name (not a path)</param>
    void SetOUTNAME(std::string name = u8"simDEBUG.RO.dbg");

    /// <summary>
    /// Print all output
    /// </summary>
    void PrintAllOut(std::ostream& ssout = std::cout) const;

  private:
    static constexpr size_t MAX_SWAP = 500;

    float avrSWAP[MAX_SWAP];
    int aviFAIL;
    char accINFILE[4096];
    char avcOUTNAME[1024];
    char avcMSG[4096];
};

/**
 * @brief Controller using DISCON routine.
 */
class ControllerDISCON : public Controller {
  public:
    /** @brief Object for communication with DISCON routine. */
    seahowl::servo::DisconController pImpl;

    /**
     * @brief Constructor.
     *
     * @param[in] infile Path of parameters file.
     * @param[in] infile Path of output file.
     */
    ControllerDISCON(std::string infile = u8"DISCON.IN", std::string libfile = u8"libdiscon.so");

    /**
     * @brief Initialization of controller.
     *
     * @param[in] time Time of simulation.
     * @param[in] dt Time step legnth.
     * @param[in] turbine Turbine that is controlled by this controller.
     */
    virtual void initialize(double time, double dt, const seahowl::core::Turbine& turbine) override;
    /**
     * @brief Stepping of controller.
     *
     * @param[in] time Time of simulation.
     * @param[in] dt Time step legnth.
     * @param[in] turbine Turbine that is controlled by this controller.
     */
    virtual void step(double time, double dt, const seahowl::core::Turbine& turbine) override;

    /**
     * @brief Returns electrical torque to apply.
     */
    virtual double get_torque_elec() const override;

    /**
     * @brief Returns collective pitch to apply.
     */
    virtual double get_collective_pitch() const override;

    /**
     * @brief Returns pitch to apply on blade.
     *
     * @param[in] index_blade Index of blade (0, 1, or 2).
     */
    virtual double get_pitch_blade(int index_blade) const override;

  private:
    /** @brief Filepath of dynamic library (for DISCON routine). */
    std::string libfile;

    // updates turbine variables of object communicating with DISCON module
    void update_turbine_variables(double time, double dt, const seahowl::core::Turbine& turbine);
};

}  // namespace servo
}  // namespace seahowl
