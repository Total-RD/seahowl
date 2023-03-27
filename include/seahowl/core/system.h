#pragma once
#include <seahowl/core/turbine.h>
#include <seahowl/aero/wind_models.h>
#include <seahowl/servo/controller.h>
#include <seahowl/servo/controller_discon.h>
#include <seahowl/elasto/entities_elasto.h>
#include <seahowl/elasto/system_elasto.h>
#include <seahowl/aero/system_aero.h>

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
    std::deque<Turbine> turbines{};
    /** @brief Wind model. */
    std::shared_ptr<seahowl::aero::WindModel> wind_model;
    std::shared_ptr<seahowl::elasto::SystemElasto> system_elasto;
    std::shared_ptr<seahowl::aero::SystemAero> system_aero;

    System();

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
    double get_time();
};
}  // namespace core
}  // namespace seahowl
