#pragma once

#include <vector>
#include <memory>
#include "seahowl/env/fluid_models.h"
#include "seahowl/env/list_model.h"

namespace seahowl {
namespace env {
/**
 * @brief Class to store list of fluids
 */
class FluidListModel : public ListModel<FluidModel> {
  public:
    /**
     * @brief Returns fluid density at given coordinates.
     *
     * @param[in] position Position at which fluid density is extracted.
     * @param[in] time Time of simulation.
     */
    double get_density(const Vector3d& position, double time) const;

    /**
     * @brief Returns fluid velocity at given coordinates.
     *
     * @param[in] position Position at which fluid velocity is extracted.
     * @param[in] time Time of simulation.
     */
    Vector3d get_velocity(const Vector3d& position, double time) const;

    /**
     * @brief Returns fluid acceleration at given coordinates.
     *
     * @param[in] position Position at which fluid acceleration is extracted.
     * @param[in] time Time of simulation.
     */
    Vector3d get_acceleration(const Vector3d& position, double time) const;
};

}  // namespace env
}  // namespace seahowl
