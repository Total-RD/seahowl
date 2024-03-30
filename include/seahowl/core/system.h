#pragma once

#include "seahowl/core/component.h"
#include "seahowl/core/turbine.h"  // @todo forward declare Turbine

#include <deque>

// forward declarations
namespace seahowl {
namespace env {
class FluidModel;
class SoilModel;
}  // namespace env
namespace servo {
class Controller;
}  // namespace servo
namespace aero {
class SystemAero;
}  // namespace aero
namespace elasto {
class SystemElasto;
}  // namespace elasto
}  // namespace seahowl

namespace seahowl {
namespace core {

/**
 * @brief System composed of wind turbines and environmental conditions.
 *
 * This class controls the workflow between wind turbines and the environment (wind, waves, etc).
 */
class System : public ComponentDynamic {
  public:
    /** @brief Wind turbines. */
    std::deque<std::shared_ptr<Turbine>> turbines{};
    /** @brief Fluid model. */
    std::shared_ptr<seahowl::env::FluidModel> fluid_model;
    /** @brief Soil model. */
    std::shared_ptr<seahowl::env::SoilModel> soil_model;
    /** @brief System for elastodynamics. */
    seahowl::elasto::SystemElasto& elasto;
    /** @brief System for aerodynamics. */
    seahowl::aero::SystemAero& aero;
    /**
     * @brief Constructor.
     *
     * Instantiates system for communication between elasto, aero, servo, hydro components.
     *
     * @param[in] elasto Elastodynamic system.
     * @param[in] aero Aerodynamic system.
     */
    System(seahowl::elasto::SystemElasto& elasto, seahowl::aero::SystemAero& aero);

    /**
     * @brief Initialize system.
     *
     * @param[in] time Time of the simulation (usually 0 at init).
     * @param[in] dt Time step length.
     */
    virtual void initialize(double time, double dt) override;

    /**
     * @brief Prestep for system, called before elastodynamic stepping.
     *
     * @param[in] time Time of the simulation.
     * @param[in] dt Time step length.
     */
    virtual void prestep(double time, double dt) override;

    /**
     * @brief Step for system, called for elastodynamic stepping.
     *
     * @param[in] time Time of the simulation.
     */
    void step(double dt);

    /**
     * @brief Poststep for system, called after elastodynamic stepping.
     *
     * @param[in] time Time of the simulation.
     * @param[in] dt Time step length.
     */
    virtual void poststep(double time, double dt) override;

    /**
     * @brief Assembles the turbine (elasto part).*
     *
     * Calls assemble for each of the components of the turbine.
     *
     * @param[out] system System on which to add bodies, links, etc.
     * @param[out] mesh Mesh on which to add nodes and elements.
     */
    void assemble();

    /**
     * @brief Returns time of simulation.
     */
    double get_time() const;

    /**
     * @brief Sets time of simulation.
     *
     * @param[in] time Time of simulation.
     */
    void set_time(double time);

    /**
     * @brief Presetup for system, called before simulation actually starts.
     *
     * @param[in] presetup_duration Duration of presetup.
     * @param[in] presetup_dt Time step length.
     */
    void run_presetup(double presetup_duration, double presetup_dt);

    /**
     * @brief Presimulation for system, called before simulation actually starts.
     * All external loads are applied using initial conditions for the whole duration of the presimulation.
     *
     * @param[in] presim_duration Duration of presimulation.
     * @param[in] presim_dt Time step length.
     * @param[in] fix_towers Whether to fix tower bases or not.
     */
    void run_presimulation(double presim_duration, double presim_dt, bool fix_towers = true);

    /**
     * @brief Adds turbine to system.
     *
     * @param[in] turbine Turbine to add to system.
     */
    void add_turbine(std::shared_ptr<seahowl::core::Turbine> turbine);
};
}  // namespace core
}  // namespace seahowl
