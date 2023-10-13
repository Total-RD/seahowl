#pragma once

#include "seahowl/core/turbine.h"
#include "seahowl/elasto/turbine_floating_elasto.h"
#include "seahowl/elasto/floater_elasto.h"
#include "seahowl/elasto/entities_elasto.h"

/**@brief Seahowl base namespace */
namespace seahowl {

/**@brief Seahowl core module */
namespace core {

/**
 * @brief Floating Offshore Wind Turbine (FOWT).
 *
 * This class controls each component, ensuring proper workflow and communication within and between the component.
 */
class TurbineFloating : public Turbine {
  public:
    /** @brief Elastodynamic model of the turbine. */
    seahowl::elasto::TurbineFloatingElasto& elasto;

    /**
     * @brief Constructor.
     *
     * Instantiates turbine for communication between elasto and aero components.
     *
     * @param[in] elasto Elastodynamic turbine model.
     * @param[in] aero Aerodynamic turbine model.
     */
    TurbineFloating(seahowl::elasto::TurbineFloatingElasto& elasto, seahowl::aero::TurbineAero& aero);

    /**
     * @brief Initialize turbine, called before starting the simulation.
     *
     * Calls init for each of its components.
     *
     * @param[in] time Time of the simulation (usually 0 at init).
     * @param[in] dt Time step length.
     */
    void initialize(double time, double dt) override;

    /**
     * @brief Prestep for turbine, called before elastodynamic stepping.
     *
     * Calls prestep on each of the components of the turbine.
     *
     * @param[in] time Time of the simulation.
     * @param[in] dt Time step length.
     */
    void prestep(double time, double dt) override;

    /**
     * @brief Poststep for turbine, called after elastodynamic stepping.
     *
     * Applies step for the controller (potentially modifying loads with electrical torque and elasto positions due to
     * blade pitching), and then calls poststep on each of the components of the turbine.
     *
     * @param[in] time Absolute time of the simulation.
     * @param[in] dt Time step length.
     */
    void poststep(double time, double dt) override;

    /**
     * @brief Builds the turbine.
     *
     * Calls build for each of the components of the turbine.
     */
    void build();
};

}  // namespace core
}  // namespace seahowl
