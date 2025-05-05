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

    /**
     * @brief set ramp time for all fluid model.
     * @param[in] start_time Start time of the ramp.
     * @param[in] end_time End time of the ramp.
     */
    void set_ramp(double start_time, double end_time);
};

}  // namespace env
}  // namespace seahowl
