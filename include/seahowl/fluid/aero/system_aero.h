#pragma once

#include "seahowl/fluid/aero/turbine_aero.h"  // @todo forward declare
#include "seahowl/commons/component_fluid.h"

#include <vector>
#include <deque>

namespace seahowl {
namespace aero {

/**
 * @brief Aero system base class.
 */
class SystemAero : public ComponentFluid {
  public:
    /** @brief Turbines in system. */
    std::deque<std::shared_ptr<TurbineAero>> turbines{};
    /** @brief Components in system. */
    std::deque<std::shared_ptr<ComponentFluid>> components{};

    /**
     * @brief Builds the tower.
     */
    void build() override;

    /**
     * @brief Computes aero loads on sytem.
     *
     * @param[in] env_model env model to use for applying aero loads.
     * @param[in] time Time of simulation.
     */
    void compute_env_loads(const env::EnvModel& env_model, double time) override;

    /**
     * @brief Adds turbine to system.
     *
     * @param[in] turbine Turbine to add to system.
     */
    void add(std::shared_ptr<TurbineAero> turbine) { turbines.push_back(turbine); }

    /**
     * @brief Adds component to system.
     *
     * @param[in] component Component to add to system.
     */
    void add(std::shared_ptr<seahowl::ComponentFluid> component) { components.push_back(component); }
};

}  // namespace aero
}  // namespace seahowl
