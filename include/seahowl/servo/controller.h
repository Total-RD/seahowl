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

// Standard library
#include <string>

// forward declarations
namespace seahowl {
namespace core {
class Turbine;
}  // namespace core
}  // namespace seahowl

namespace seahowl {
/** @brief Servo controller module. */
namespace servo {

/**
 * @brief Base class for controller.
 */
class Controller {
  public:
    /** @brief Whether pitch control is applied or not. */
    bool has_pitch_control = false;
    /** @brief Whether torque control is applied or not. */
    bool has_torque_control = false;
    /** @brief Whether yaw control is applied or not. */
    bool has_yaw_control = false;
    /** @brief Output folder for controller. */
    std::string output_folder = "./output";

    /**
     * @brief Constructor.
     */
    Controller();

    /**
     * @brief Initialization of controller.
     *
     * @param[in] time Time of simulation [s]
     * @param[in] dt Time step length [s]
     * @param[in] turbine Turbine that is controlled by this controller.
     */
    virtual void initialize(double time, double dt, const seahowl::core::Turbine& turbine);

    /**
     * @brief Stepping of controller.
     *
     * @param[in] time Time of simulation [s]
     * @param[in] dt Time step length [s]
     * @param[in] turbine Turbine that is controlled by this controller.
     */
    virtual void step(double time, double dt, const seahowl::core::Turbine& turbine);

    /**
     * @brief Post-step for controller.
     *
     * @param[in] time Time of simulation [s]
     * @param[in] dt Time step length [s]
     * @param[in] turbine Turbine that is controlled by this controller.
     */
    virtual void poststep(double time, double dt, const seahowl::core::Turbine& turbine);

    /**
     * @brief Returns electrical torque to apply.
     *
     * @return Electrical torque [Nm]
     */
    virtual double get_torque_elec() const;

    /**
     * @brief Returns collective pitch to apply.
     *
     * @return Collective pitch [rad]
     */
    virtual double get_collective_pitch() const;

    /**
     * @brief Returns pitch to apply on blade.
     *
     * @param[in] index_blade Index of blade (0, 1, or 2).
     * @return Blade pitch [rad]
     */
    virtual double get_pitch_blade(int index_blade) const;

    /**
     * @brief Returns yaw rate to apply to yaw bearing.
     *
     * @return Yaw rate [rad/s]
     */
    virtual double get_yaw_rate() const;
};

/**
 * @brief Controller using electrical torque to reach a target (max) RPM.
 */
class ControllerVariableTorque : public Controller {
  private:
    /** @brief Current electrical torque [Nm] */
    double torque_elec = 0.0;
    /** @brief Previous electrical torque [Nm] */
    double torque_elec_previous = 0.0;
    /** @brief Target RPM (max RPM for turbine) [rpm] */
    double target_rpm = 0.0;

  public:
    /**
     * @brief Constructor.
     */
    ControllerVariableTorque();

    virtual void step(double time, double dt, const seahowl::core::Turbine& turbine) override;
    virtual void poststep(double time, double dt, const seahowl::core::Turbine& turbine) override;
    virtual double get_torque_elec() const override;

    /**
     * @brief Sets target RPM.
     *
     * @param[in] target_rpm Target (max) RPM [rpm]
     */
    void set_target_rpm(double target_rpm);
};

}  // namespace servo
}  // namespace seahowl
