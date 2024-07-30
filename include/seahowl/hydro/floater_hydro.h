#pragma once

#include <memory>

// forward declarations
namespace seahowl {
namespace hydro {
class MooringSystemHydro;
}  // namespace hydro
}  // namespace seahowl

namespace seahowl {

/**@brief Hydrodynamic module */
namespace hydro {

/**
 * @brief Floater of wind turbine as an hydrodynamic component.
 */
class FloaterHydro {
  public:
    /** @brief Mooring system of the floater. */
    std::unique_ptr<MooringSystemHydro> mooring_system;

    /**
     * @brief Constructor.
     */
    FloaterHydro();
};

}  // namespace hydro
}  // namespace seahowl
