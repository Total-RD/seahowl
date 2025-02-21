#pragma once

#include "seahowl/core/component.h"
#include "seahowl/core/turbine.h"  // @todo forward declare Turbine

#include <deque>

// forward declarations
namespace seahowl {
namespace env {
class EnvModel;
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
    /** @brief Other dynamics components. */
    std::deque<std::shared_ptr<ComponentDynamic>> components{};
    /** @brief environmental model. */
    std::shared_ptr<seahowl::env::EnvModel> env_model;
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
    System(std::shared_ptr<seahowl::elasto::SystemElasto> elasto, std::shared_ptr<seahowl::aero::SystemAero> aero);

    void build() override;

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
     * @brief Applies environmental model to system.
     * @param[in] env_model Environmental model affecting system.
     * @param[in] time Time of simulation.
     */
    void apply_env_model(seahowl::env::EnvModel& env_model, double time) override;

    void apply_soil_model(seahowl::env::EnvModel& env_model, double time) override;

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
     * @brief Presimulation for system, called before simulation actually starts.
     *
     * @param[in] duration Duration of presimulation.
     * @param[in] dt Time step length.
     * @param[in] fix_towers Whether to fix tower bases or not.
     * @param[in] with_presetup Whether to do presetup or not.
     */
    void run_presimulation(double duration, double dt, bool fix_towers = true, bool with_presetup = true);

    /**
     * @brief Adds turbine to system.
     *
     * @param[in] turbine Turbine to add to system.
     */
    void add(std::shared_ptr<seahowl::core::Turbine> turbine);

    /**
     * @brief Adds component to system.
     *
     * @param[in] component Component to add to system.
     */
    void add(std::shared_ptr<seahowl::core::ComponentDynamic> component);

  private:
    std::shared_ptr<seahowl::elasto::SystemElasto> elasto_ptr;
    std::shared_ptr<seahowl::aero::SystemAero> aero_ptr;

    /**
     * @brief Initialize system.
     *
     * @param[in] time Time of the simulation (usually 0 at init).
     * @param[in] dt Time step length.
     */
    virtual void initialize_this(double time, double dt) override;
};
}  // namespace core
}  // namespace seahowl
