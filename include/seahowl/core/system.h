#pragma once
#include <seahowl/core/turbine.h>
#include <seahowl/aero/wind_models.h>
#include <seahowl/servo/controller.h>
#include <seahowl/servo/controller_discon.h>
#include <chrono/physics/ChBody.h>

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
    std::vector<Turbine> turbines{};
    /** @brief Wind model. */
    std::shared_ptr<seahowl::aero::WindModel> wind_model;

    System();

    /**
     * @brief Initialize system.
     *
     * @param[in] time Time of the simulation (usually 0 at init).
     * @param[in] dt Time step length.
     */
    virtual void init(double time, double dt) override;

    /**
     * @brief Prestep for system, called before elastodynamic stepping.
     *
     * @param[in] time Time of the simulation.
     * @param[in] dt Time step length.
     */
    virtual void prestep(double time, double dt) override;

    /**
     * @brief Poststep for system, called after elastodynamic stepping.
     *
     * @param[in] time Time of the simulation.
     * @param[in] dt Time step length.
     */
    virtual void poststep(double time, double dt) override;
};
}  // namespace core
}  // namespace seahowl
