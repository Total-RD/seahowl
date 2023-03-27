#pragma once

#include <seahowl/aero/turbine_aero.h>

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
    std::deque<TurbineAero> turbines{};
};

}  // namespace aero
}  // namespace seahowl
