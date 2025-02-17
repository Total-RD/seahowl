#pragma once

#include "seahowl/core/tower.h"
#include "seahowl/core/foundation.h"

namespace seahowl {
namespace core {

/**
 * @brief Monopile of wind turbine.
 */
class Monopile : public Tower, public Foundation {
  public:
    /**
     * @brief Instantiates tower for communication between elasto and aero components.
     *
     * @param[in] elasto Elastodynamic tower model.
     * @param[in] aero Aerodynamic tower model.
     */
    Monopile(seahowl::elasto::TowerElasto& elasto, seahowl::aero::TowerAero& aero) : Tower(elasto, aero){};
};

}  // namespace core
}  // namespace seahowl
