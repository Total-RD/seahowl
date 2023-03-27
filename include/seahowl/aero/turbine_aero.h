#pragma once

#include <seahowl/aero/blade_aero.h>
#include <seahowl/aero/rotor_aero.h>
#include <seahowl/aero/tower_aero.h>
#include <seahowl/commons/numerics.h>
#include <seahowl/commons/entities.h>

#include <vector>

/**@brief Seahowl base namespace */
namespace seahowl {

/**@brief Seahowl aero module */
namespace aero {

class ComponentAero {
    virtual void compute_aero_loads(seahowl::aero::WindModel& wind_model, double time){};
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
    seahowl::aero::RotorAero rotor;
    /** @brief Tower of the turbine. */
    seahowl::aero::TowerAero tower;

    /**
     * @brief Constructor.
     *
     * Instantiates rotor component and tower component.
     */
    TurbineAero();

    void build();

    /**
     * @brief Computes wind loads on all aero nodes of blades.
     *
     * @param[in] wind_model Wind model to use for retrieving uninduced wind velocity at nodes.
     * @param[in] time Time of simulation.
     * @param[in] tower_shadow Whether to take tower shadow effect into account or not.
     * @param[in] tip_loss Whether to take tip loss into account or not.
     * @param[in] hub_loss Whether to take hub loss into account or not.
     */
    // virtual void compute_aero_loads(seahowl::aero::WindModel& wind_model, double time) override;
};

}  // namespace aero
}  // namespace seahowl
