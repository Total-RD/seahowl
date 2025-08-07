#pragma once

#include "seahowl/core/tower.h"
#include "seahowl/elasto/monopile_elasto.h"
#include "seahowl/fluid/hydro/monopile_hydro.h"
#include "seahowl/core/foundation.h"

namespace seahowl {
namespace core {

/**
 * @brief Monopile of wind turbine.
 */
class Monopile : public Tower, public virtual Foundation {
  public:
    /** @brief Elastodynamic model of the monopile. */
    seahowl::elasto::MonopileElasto& elasto;
    /** @brief Aerodynamic model of the monopile. */
    seahowl::hydro::MonopileHydro& hydro;

    /**
     * @brief Instantiates tower for communication between elasto and aero components.
     *
     * @param[in] elasto Elastodynamic tower model.
     * @param[in] aero Aerodynamic tower model.
     */
    Monopile(std::shared_ptr<seahowl::elasto::MonopileElasto> elasto,
             std::shared_ptr<seahowl::hydro::MonopileHydro> hydro)
        : Foundation(elasto, hydro),
          Tower(elasto, hydro),
          ComponentDynamic(elasto, hydro),
          elasto(*elasto),
          hydro(*hydro){};
};

}  // namespace core
}  // namespace seahowl
