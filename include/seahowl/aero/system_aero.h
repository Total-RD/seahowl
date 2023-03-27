#pragma once

#include <seahowl/aero/turbine_aero.h>

#include <vector>

namespace seahowl {
namespace aero {

/**
 * @brief Aero system base class.
 */
class SystemAero {
  public:
    std::vector<TurbineAero> turbines;
};

}  // namespace aero
}  // namespace seahowl
