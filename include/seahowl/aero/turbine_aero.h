#pragma once

#include "seahowl/aero/rotor_aero.h"
#include "seahowl/aero/tower_aero.h"
#include "seahowl/hydro/foundation_fluid.h"
#include "seahowl/hydro/floater_hydro.h"
#include "seahowl/commons/component_fluid.h"

#include <vector>

// forward declarations
namespace seahowl {
namespace env {
class WindModel;
}  // namespace env
}  // namespace seahowl

/**@brief Seahowl base namespace */
namespace seahowl {

/**@brief Seahowl aero module */
namespace aero {
/**
 * @brief Wind turbine (blades, rotor-nacelle assembly, tower).
 *
 * This class controls each component, ensuring proper workflow for the aero part.
 */
class TurbineAero : public ComponentFluid {
  public:
    // components
    //
    /** @brief Rotor-nacelle assembly of the turbine. */
    seahowl::aero::RotorNacelleAssemblyAero rna;
    /** @brief Tower of the turbine. */
    seahowl::aero::TowerAero tower;
    /** @brief Foundation of the turbine. */
    std::shared_ptr<seahowl::hydro::FoundationFluid> foundation;

    /**
     * @brief Constructor.
     *
     * Instantiates rotor component and tower component.
     */
    TurbineAero();

    /**
     * @brief Builds turbine.
     */
    void build() override;

    /**
     * @brief Initializes turbine.
     *
     * @param[in] time Time of the simulation (usually 0 at init).
     * @param[in] dt Time step length.
     */
    virtual void initialize(double time, double dt);

    /**
     * @brief Computes aero loads on turbine.
     *
     * @param[in] wind_model Wind model to use for applying aero loads.
     * @param[in] time Time of simulation.
     */
    virtual void compute_fluid_loads(const env::EnvModel& fluid_model, double time) override;
};

}  // namespace aero
}  // namespace seahowl
