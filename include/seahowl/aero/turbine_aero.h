#pragma once

#include "seahowl/aero/rotor_aero.h"
#include "seahowl/aero/tower_aero.h"
#ifdef HAVE_AERODYN
    #include "seahowl/aero/aerodyn_adapter.h"
#endif

#include <vector>

// forward declarations
namespace seahowl {
namespace aero {}  // namespace aero
}  // namespace seahowl

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
    seahowl::aero::RotorNacelleAssemblyAero rotor;
    /** @brief Tower of the turbine. */
    seahowl::aero::TowerAero tower;

    /** @brief Whether to use AeroDyn or not. */
    bool use_aerodyn = false;
#ifdef HAVE_AERODYN
    /** @brief AeroDyn adapter (only used if AeroDyn is enabled). */
    std::shared_ptr<seahowl::aero::AeroDynAdapter> aerodyn;
    /** @brief Option to save VTK in AeroDyn, 0: none; 1: init only; 2: animation. */
    int WrVTK = 0;
    /** @brief VTK save type, 1: surface; 2: lines; 3: both. */
    int WrVTK_Type = 1;
    /** @brief VTK save time step. */
    double WrVTK_dt;
#endif

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
    void initialize(double time, double dt);

    /**
     * @brief Computes aero loads on turbine.
     *
     * @param[in] wind_model Wind model to use for applying aero loads.
     * @param[in] time Time of simulation.
     */
    virtual void compute_aero_loads(seahowl::aero::WindModel& wind_model, double time) override;
};

}  // namespace aero
}  // namespace seahowl
