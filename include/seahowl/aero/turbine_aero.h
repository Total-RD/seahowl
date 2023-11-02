#pragma once

#include "seahowl/aero/rotor_aero.h"
#include "seahowl/aero/tower_aero.h"

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

class ComponentAero {
    virtual void compute_aero_loads(const env::FluidModel& wind_model, double time){};
};

/**
 * @brief Wind turbine (blades, rotor-nacelle assembly, tower).
 *
 * This class controls each component, ensuring proper workflow for the aero part.
 */
class TurbineAero : public ComponentAero {
  public:
    // components
    //
    /** @brief Rotor-nacelle assembly of the turbine. */
    seahowl::aero::RotorNacelleAssemblyAero rna;
    /** @brief Tower of the turbine. */
    seahowl::aero::TowerAero tower;

    /**
     * @brief Constructor.
     *
     * Instantiates rotor component and tower component.
     */
    TurbineAero();

    /**
     * @brief Builds turbine.
     */
    void build();

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
    virtual void compute_aero_loads(const env::FluidModel& wind_model, double time) override;
};

}  // namespace aero
}  // namespace seahowl
