#pragma once

#include "seahowl/aero/turbine_aero.h"  // @todo forward declare
#include "seahowl/commons/component_fluid.h"

#include <vector>
#include <deque>

namespace seahowl {
namespace aero {

/**
 * @brief Aero system base class.
 */
class SystemAero {
  public:
    /** @brief Turbines in system. */
    std::deque<std::shared_ptr<TurbineAero>> turbines{};
    /** @brief Components in system. */
    std::deque<std::shared_ptr<ComponentFluid>> components{};

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
