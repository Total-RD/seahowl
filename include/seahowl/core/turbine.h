#pragma once

#include "seahowl/core/component.h"
#include "seahowl/core/rotor.h"
#include "seahowl/core/tower.h"

#include <vector>

// forward declarations
namespace seahowl {
namespace env {
class SoilModel;
class FluidModel;
}  // namespace env
namespace servo {
class Controller;
}  // namespace servo
namespace aero {
class TurbineAero;
}  // namespace aero
namespace elasto {
class TurbineElasto;
}  // namespace elasto
}  // namespace seahowl

/**@brief Seahowl base namespace */
namespace seahowl {

/**@brief Seahowl core module */
namespace core {

/**
 * @brief Wind turbine (blades, rotor-nacelle assembly, tower).
 *
 * This class controls each component, ensuring proper workflow and communication within and between the component.
 */
class Turbine : public ComponentDynamic {
  public:
    // components
    //
    /** @brief Elastodynamic model of the turbine. */
    seahowl::elasto::TurbineElasto& elasto;
    /** @brief Aerodynamic model of the turbine. */
    seahowl::aero::TurbineAero& aero;
    /** @brief Rotor-nacelle assembly of the turbine. */
    RotorNacelleAssembly rna;
    /** @brief Tower of the turbine. */
    Tower tower;
    /** @brief Controller of the turbine. */
    std::shared_ptr<seahowl::servo::Controller> controller;

    // parameters
    //
    /** @brief Efficiency of the generator. */
    double generator_efficiency = 1.0;
    /** @brief Ratio of the gearbox. */
    double gearbox_ratio = 1.0;
    /** @brief Efficiency of the gearbox. */
    double gearbox_efficiency = 1.0;

    /**
     * @brief Constructor.
     *
     * Instantiates turbine for communication between elasto and aero components.
     *
     * @param[in] elasto Elastodynamic turbine model.
     * @param[in] aero Aerodynamic turbine model.
     */
    Turbine(seahowl::elasto::TurbineElasto& elasto, seahowl::aero::TurbineAero& aero);

    /**
     * @brief Initialize turbine, called before starting the simulation.
     *
     * Calls init for each of its components.
     *
     * @param[in] time Time of the simulation (usually 0 at init).
     * @param[in] dt Time step length.
     */
    virtual void initialize(double time, double dt) override;

    /**
     * @brief Prestep for turbine, called before elastodynamic stepping.
     *
     * Calls prestep on each of the components of the turbine.
     *
     * @param[in] time Time of the simulation.
     * @param[in] dt Time step length.
     */
    void prestep(double time, double dt) override;

    /**
     * @brief Poststep for turbine, called after elastodynamic stepping.
     *
     * Applies step for the controller (potentially modifying loads with electrical torque and elasto positions due to
     * blade pitching), and then calls poststep on each of the components of the turbine.
     *
     * @param[in] time Absolute time of the simulation.
     * @param[in] dt Time step length.
     */
    void poststep(double time, double dt) override;

    /**
     * @brief Builds the turbine.
     *
     * Calls build for each of the components of the turbine.
     */
    void build();

    /**
     * @brief Translates the turbine.
     *
     * @param[in] translation_vector The 3D translation vector.
     */
    void translate(Vector3d translation_vector);

    /**
     * @brief Rotates the turbine.
     *
     * @param[in] translation_vector The angle of rotation (in radians).
     * @param[in] axis The axis of rotation (3D vector).
     */
    void rotate(double angle, Vector3d axis);

    /**
     * @brief Returns shaft power.
     */
    double get_shaft_power() const;

    /**
     * @brief Returns generated power.
     */
    double get_generated_power() const;

    /**
     * @brief Returns generator RPM.
     */
    double get_generator_rpm() const;

    /**
     * @brief Applies fluid model to turbine components.
     *
     * @param[in] fluid_model Fluid model affecting turbine components.
     * @param[in] time Time of simulation.
     */
    virtual void apply_fluid_model(seahowl::env::FluidModel& fluid_model, double time);

    /**
     * @brief Applies soil model to turbine components.
     *
     * @param[in] soil_model Soil model affecting turbine components.
     * @param[in] time Time of simulation.
     */
    virtual void apply_soil_model(seahowl::env::SoilModel& soil_model, double time);
};

}  // namespace core
}  // namespace seahowl
