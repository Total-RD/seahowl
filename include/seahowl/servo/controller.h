#pragma once

// forward declarations
namespace seahowl {
namespace core {
class Turbine;
}  // namespace core
}  // namespace seahowl

namespace seahowl {
/**@brief Servo controller module */
namespace servo {

/**
 * @brief Base class for controller.
 */
class Controller {
  public:
    /** @brief Whether pitch control is applied or not. */
    bool has_pitch_control;
    /** @brief Whether torque control is applied or not. */
    bool has_torque_control;

    /**
     * @brief Constructor.
     */
    Controller();

    /**
     * @brief Initialization of controller.
     *
     * @param[in] time Time of simulation.
     * @param[in] dt Time step legnth.
     * @param[in] turbine Turbine that is controlled by this controller.
     */
    virtual void initialize(double time, double dt, const seahowl::core::Turbine& turbine);

    /**
     * @brief Stepping of controller.
     *
     * @param[in] time Time of simulation.
     * @param[in] dt Time step legnth.
     * @param[in] turbine Turbine that is controlled by this controller.
     */
    virtual void step(double time, double dt, const seahowl::core::Turbine& turbine);

    /**
     * @brief Post-step for controller.
     *
     * @param[in] time Time of simulation.
     * @param[in] dt Time step legnth.
     * @param[in] turbine Turbine that is controlled by this controller.
     */
    virtual void poststep(double time, double dt, const seahowl::core::Turbine& turbine);

    /**
     * @brief Returns electrical torque to apply.
     */
    virtual double get_torque_elec() const;

    /**
     * @brief Returns collective pitch to apply.
     */
    virtual double get_collective_pitch() const;

    /**
     * @brief Returns pitch to apply on blade.
     *
     * @param[in] index_blade Index of blade (0, 1, or 2).
     */
    virtual double get_pitch_blade(int index_blade) const;
};

/**
 * @brief Controller using electrical torque to reach a target (max) RPM.
 */
class ControllerVariableTorque : public Controller {
  private:
    /** @brief Current electrical torque. */
    double torque_elec = 0.0;
    /** @brief Previous electrical torque. */
    double torque_elec_previous = 0.0;

  public:
    /** @brief Target RPM (max RPM for turbine). */
    double target_rpm = 0.0;

    /**
     * @brief Constructor.
     */
    ControllerVariableTorque();

    /**
     * @brief Stepping of controller.
     *
     * @param[in] time Time of simulation.
     * @param[in] dt Time step legnth.
     * @param[in] turbine Turbine that is controlled by this controller.
     */
    virtual void step(double time, double dt, const seahowl::core::Turbine& turbine) override;

    /**
     * @brief Post-step for controller.
     *
     * @param[in] time Time of simulation.
     * @param[in] dt Time step legnth.
     * @param[in] turbine Turbine that is controlled by this controller.
     */
    virtual void poststep(double time, double dt, const seahowl::core::Turbine& turbine) override;

    /**
     * @brief Returns electrical torque to apply.
     */
    virtual double get_torque_elec() const override;
};

}  // namespace servo
}  // namespace seahowl
