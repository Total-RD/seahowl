#pragma once

#include <memory>
#include <vector>
#include <stdexcept>
#include <algorithm>

#include "seahowl/commons/entities.h"

// forward declarations
namespace seahowl {
class ComponentFluid;
namespace env {
class EnvModel;
}  // namespace env
namespace elasto {
class ComponentElasto;
}
}  // namespace seahowl

namespace seahowl {
namespace core {

/**
 * @brief Component that evolves dynamically during simulation.
 */
class ComponentDynamic {
  public:
    ComponentDynamic(const std::shared_ptr<seahowl::elasto::ComponentElasto> elasto_,
                     const std::shared_ptr<seahowl::ComponentFluid> fluid_)
        : elasto_ptr(elasto_), fluid_ptr(fluid_) {}

    /**
     * @brief Builds the component, called before initializing the simulation.
     */
    virtual void build() = 0;

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
     * @brief Applies env model to component.
     *
     * @param[in] env_model env model affecting component.
     * @param[in] time Time of simulation.
     */
    virtual void apply_env_model(seahowl::env::EnvModel& env_model, double time){};

    /**
     * @brief Applies soil model to component.
     *
     * @param[in] env_model env model affecting component.
     * @param[in] time Time of simulation.
     */
    virtual void apply_soil_model(seahowl::env::EnvModel& env_model, double time){};

    /**
     * @brief Get elasto shared_ptr component.
     */
    std::shared_ptr<seahowl::elasto::ComponentElasto> get_shared_elasto() const { return elasto_ptr; }

    /**
     * @brief Get fluid shared_ptr component.
     */
    std::shared_ptr<seahowl::ComponentFluid> get_shared_fluid() const { return fluid_ptr; }

  protected:
    bool is_initialized = false;

  private:
    // Only for memory management, never accessed (reference to underlying object is accessed instead).
    std::shared_ptr<seahowl::elasto::ComponentElasto> elasto_ptr;
    // Only for memory management, never accessed (reference to underlying object is accessed instead).
    std::shared_ptr<seahowl::ComponentFluid> fluid_ptr;

    virtual void initialize_this(double time, double dt) = 0;
};

}  // namespace core
}  // namespace seahowl
