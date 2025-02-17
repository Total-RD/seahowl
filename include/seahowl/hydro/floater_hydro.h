#pragma once

#include "seahowl/hydro/foundation_fluid.h"

#include <memory>

// forward declarations
namespace seahowl {
namespace hydro {
class MooringSystemHydro;
}  // namespace hydro
namespace env {
class FluidModel;
}  // namespace env
}  // namespace seahowl

namespace seahowl {

/**@brief Hydrodynamic module */
namespace hydro {

/**
 * @brief Floater of wind turbine as an hydrodynamic component.
 */
class FloaterHydro : public FoundationFluid {
  public:
    /** @brief Mooring system of the floater. */
    std::unique_ptr<MooringSystemHydro> mooring_system;

    /**
     * @brief Constructor.
     */
    FloaterHydro();

    void build() override;

    void compute_fluid_loads(const env::FluidModel& fluid_model, double time) override;
};

}  // namespace hydro
}  // namespace seahowl
