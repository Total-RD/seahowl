#pragma once

#include <vector>
#include <stdexcept>
#include <algorithm>

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

  protected:
    bool is_initialized = false;

  private:
    virtual void initialize_this(double time, double dt) = 0;
};

}  // namespace core
}  // namespace seahowl
