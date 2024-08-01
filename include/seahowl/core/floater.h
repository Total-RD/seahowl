#pragma once

#include <memory>
#include <vector>

#include "seahowl/commons/utils.h"  // for DiscretizationPoint
#include "seahowl/core/component.h"
#include "seahowl/core/mooring.h"

// forward declarations
namespace seahowl {
namespace core {
class MooringSystem;
}  // namespace core
namespace elasto {
class FloaterElasto;
}  // namespace elasto
namespace hydro {
class FloaterHydro;
}  // namespace hydro
}  // namespace seahowl

namespace seahowl {
namespace core {

class Foundation : public ComponentDynamic {};

/**
 * @brief Floater of wind turbine, with both elasto and hydro components.
 *
 */
class Floater : public Foundation {
  public:
    /** @brief Elastodynamic model of the mooring. */
    seahowl::elasto::FloaterElasto& elasto;
    /** @brief Hydrodynamic model of the mooring. */
    seahowl::hydro::FloaterHydro& hydro;
    /** @brief Mooring system of the floater. */
    std::unique_ptr<MooringSystem> mooring_system;

    /**
     * @brief Instantiates floater for communication between elasto and hydro components.
     *
     * @param[in] elasto Elastodynamic floater model.
     * @param[in] aero Hydrodynamic floater model.
     */
    Floater(seahowl::elasto::FloaterElasto& elasto, seahowl::hydro::FloaterHydro& hydro);

    /**
     * @brief Prestep for floater, called before elastodynamic stepping.
     *
     * @param[in] time Time of the simulation.
     * @param[in] dt Time step length.
     */
    void prestep(double time, double dt) override;

    /**
     * @brief Poststep for floater, called after elastodynamic stepping.
     *
     * @param[in] time Absolute time of the simulation.
     * @param[in] dt Time step length.
     */
    void poststep(double time, double dt) override;

    void apply_fluid_model(seahowl::env::FluidModel& fluid_model, double time) override;
    void apply_soil_model(seahowl::env::SoilModel& soil_model, double time) override;

    /**
     * @brief Builds the floater (hydro and elasto part).
     */
    void build() override;

  private:
    /**
     * @brief Initialize floater, called before starting the simulation.
     *
     * Runs the preset and poststep once to make elasto and aero components match.
     *
     * @param[in] time Time of the simulation (usually 0 at init).
     * @param[in] dt Time step length.
     */
    void initialize_this(double time, double dt) override;
};

}  // namespace core
}  // namespace seahowl
