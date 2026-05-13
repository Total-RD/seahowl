// SPDX-License-Identifier: Apache-2.0
// Copyright 2022–2026 TotalEnergies
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//

#pragma once

// SEAHOWL headers
#include "seahowl/servo/controller.h"

// Standard library
#include <cstring>
#include <iostream>

namespace seahowl {
namespace servo {

/** @brief DISCON wrapping (adapter) interface.
 */
class DisconInterface {
  public:
    /** @brief Time [s] */
    float& m_time = avrSWAP[1];
    /** @brief Time step [s] */
    float& m_dt = avrSWAP[2];
    /** @brief Pitch return controller states [rad] */
    float& m_pitch = avrSWAP[41];
    /** @brief Torque return controller states [Nm] */
    float& m_torque = avrSWAP[46];

    ~DisconInterface();

    /**
     * @brief Resets controller state as initial.
     */
    void ResetFirst() {
        avrSWAP[0] = 0;  // Initial step iStatus
    }

    /**
     * @brief Resets all controller fields to 0.
     */
    void ResetAll();

    /**
     * @brief Initializes the controller parameters.
     *
     * @param[in] libfile DISCON library path.
     * @param[in] tmp_folder Folder for temporary DISCON libraries.
     */
    virtual void Init(const std::string& libfile = u8"", const std::string& tmp_folder = u8"./output/tmp_discon");

    /**
     * @brief Calls the DISCON controller.
     */
    virtual void Call();

    /**
     * @brief Sets the guess pitch.
     *
     * @param[in] pitch_angle Pitch angle [rad]
     */
    void SetPitch(double pitch_angle);

    /**
     * @brief Sets blade pitch.
     *
     * @param[in] index_blade Index of blade.
     * @param[in] pitch_angle Pitch angle [rad]
     */
    void SetPitchBlade(int index_blade, double pitch_angle);

    /**
     * @brief Sets blade root moment.
     *
     * @param[in] index_blade Index of blade.
     * @param[in] flap Flapwise moment [Nm]
     * @param[in] edge Edgewise moment [Nm]
     */
    void SetRootMomentBlade(int index_blade, double flap, double edge);

    /**
     * @brief Sets tower top acceleration.
     *
     * @param[in] foreaft Fore-aft acceleration [m/s^2]
     * @param[in] sideside Side-side acceleration [m/s^2]
     */
    void SetTowerTopAcceleration(double foreaft, double sideside);

    /**
     * @brief Sets nacelle rotational acceleration.
     *
     * @param[in] roll Roll acceleration [rad/s^2]
     * @param[in] pitch Pitch acceleration [rad/s^2]
     * @param[in] yaw Yaw acceleration [rad/s^2]
     */
    void SetNacelleRotationalAcceleration(double roll, double pitch, double yaw);

    /**
     * @brief Sets inflow wind speed.
     *
     * @param[in] ws The inflow wind speed [m/s]
     */
    void SetWindSpeed(double ws);

    /**
     * @brief Sets rotor speed.
     *
     * @param[in] omega The rotor speed [rad/s]
     */
    void SetRotorSpeed(double omega);

    /**
     * @brief Sets generator speed.
     *
     * @param[in] omega The generator speed [rad/s]
     */
    void SetGeneratorSpeed(double omega);

    /**
     * @brief Sets time.
     *
     * @param[in] time The time [s]
     */
    void SetTime(double time);

    /**
     * @brief Sets time step size.
     *
     * @param[in] dt The time step size [s]
     */
    void SetDeltaTime(double dt);

    /**
     * @brief Sets rotor azimuth.
     *
     * @param[in] azimuth The rotor azimuth [rad]
     */
    void SetRotorAzimuth(double azimuth);

    /**
     * @brief Sets the generated power.
     *
     * @param[in] power The generated power [W]
     */
    void SetGeneratedPower(double power);

    /**
     * @brief Sets the shaft power.
     *
     * @param[in] power The shaft power [W]
     */
    void SetShaftPower(double power);

    /**
     * @brief Sets number of blades.
     *
     * @param[in] nblades The number of blades.
     */
    void SetNumberOfBlades(size_t nblades);

    /**
     * @brief Sets yaw error of the RNA.
     *
     * @param[in] yaw_error The yaw error [rad]
     */
    void SetYawError(double yaw_error);

    /**
     * @brief Sets value in avrSWAP array of DISCON.
     *
     * @param[in] index Index Fortran (e.g. +1 compared to C).
     * @param[in] value The value to set.
     */
    void SetAvrSWAP(size_t index, float value);

    /** @brief Sets value in avrSWAP array (forces cast of value to float). */
    void SetAvrSWAP(size_t index, size_t value);
    /** @brief Sets value in avrSWAP array (forces cast of value to float). */
    void SetAvrSWAP(size_t index, double value);

    /** @brief Forces the value of avrSWAP even if it is of "out" type. */
    void SetForcedAvrSWAP(size_t index, double value);

    /**
     * @brief Gets value from avrSWAP array of DISCON.
     *
     * @param[in] index Index Fortran (e.g. +1 compared to C).
     */
    float GetAvrSWAP(size_t index) const;

    /** @brief Gets the value of avrSWAP even if it is of "in" type. */
    float GetForcedAvrSWAP(size_t index) const;

    /**
     * @brief Sets input filename with path relative to working directory.
     *
     * Path is used for other files (e.g. control/DISCON.in finds other files in control directory).
     *
     * @param[in] name DISCON.IN input file path.
     */
    void SetINFILE(const std::string& name = u8"DISCON.IN");

    /**
     * @brief Sets output base name (relative to working directory).
     *
     * @param[in] name A name (not a path).
     */
    void SetOUTNAME(const std::string& name = u8"simDEBUG.RO.dbg");

    /**
     * @brief Prints all output.
     *
     * @param[in] ssout Output stream.
     */
    void PrintAllOut(std::ostream& ssout = std::cout) const;

  private:
    /** @brief DISCON routine type to load from dynamic library. */
    typedef void (*DISCON_routine)(float* avrSWAP, int* aviFAIL, char* accINFILE, char* avcOUTNAME, char* avcMSG);
    /** @brief DISCON routine variable loaded from dynamic library. */
    DISCON_routine DISCON;

    /** @brief Whether DLL is loaded. */
    bool has_dll = false;
    /** @brief Whether DLL has been copied. */
    bool copied_dll = false;
    /** @brief Path to DLL. */
    std::string path_dll = "";
    /** @brief DLL handler. */
    void* handler;

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
    seahowl::servo::DisconInterface pImpl;

    /**
     * @brief Constructor.
     *
     * @param[in] infile Path of parameters file.
     * @param[in] libfile Path of dynamic library file.
     */
    ControllerDISCON(const std::string& infile = u8"DISCON.IN", const std::string& libfile = u8"libdiscon.so");

    /**
     * @brief Updates turbine variables of object communicating with DISCON module.
     *
     * @param[in] time Time of simulation [s]
     * @param[in] dt Time step length [s]
     * @param[in] turbine Turbine that is controlled by this controller.
     */
    void update_turbine_variables(double time, double dt, const seahowl::core::Turbine& turbine);

    virtual void initialize(double time, double dt, const seahowl::core::Turbine& turbine) override;
    virtual void step(double time, double dt, const seahowl::core::Turbine& turbine) override;
    virtual double get_torque_elec() const override;
    virtual double get_collective_pitch() const override;
    virtual double get_pitch_blade(int index_blade) const override;
    virtual double get_yaw_rate() const override;

  private:
    /** @brief Filepath of dynamic library (for DISCON routine). */
    std::string libfile;
};

}  // namespace servo
}  // namespace seahowl
