#pragma once

#include <vector>
#include <stdexcept>
#include <algorithm>

#include "seahowl/commons/entities.h"

// forward declarations
namespace seahowl {
namespace env {
class FluidModel;
class SoilModel;
}  // namespace env
}  // namespace seahowl

namespace seahowl {
namespace core {

/**
 * @brief Component that evolves dynamically during simulation.
 */
class ComponentDynamic {
  public:
    /**
     * @brief Initializes the component, called before starting the simulation.
     *
     * @param[in] time Time of the simulation (usually 0 at init).
     * @param[in] dt Time step length.
     */
    void initialize(double time, double dt);

    /**
     * @brief Prestep for component, called before elastodynamic stepping.
     *
     * @param[in] time Time of the simulation.
     * @param[in] dt Time step length.
     */
    virtual void prestep(double time, double dt) = 0;

    /**
     * @brief Poststep for component, called after elastodynamic stepping.
     *
     * @param[in] time Time of the simulation.
     * @param[in] dt Time step length.
     */
    virtual void poststep(double time, double dt) = 0;

    /**
     * @brief Applies fluid model to component.
     *
     * @param[in] fluid_model Fluid model affecting component.
     * @param[in] time Time of simulation.
     */
    virtual void apply_fluid_model(seahowl::env::FluidModel& fluid_model, double time){};

    /**
     * @brief Applies soil model to component.
     *
     * @param[in] fluid_model Fluid model affecting component.
     * @param[in] time Time of simulation.
     */
    virtual void apply_soil_model(seahowl::env::SoilModel& soil_model, double time){};

  protected:
    bool is_initialized = false;

  private:
    virtual void initialize_this(double time, double dt) = 0;
};

}  // namespace core
}  // namespace seahowl
