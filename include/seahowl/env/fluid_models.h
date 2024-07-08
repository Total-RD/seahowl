#pragma once

#include "seahowl/commons/numerics.h"

namespace seahowl {
namespace env {

/**
 * @brief Base class for fluid models
 */
class FluidModel {
  public:
    /**
     * @brief Returns fluid velocity at given coordinates.
     *
     * @param[in] position Position at which fluid velocity is extracted.
     * @param[in] time Time of simulation.
     */
    virtual Vector3d get_fluid_velocity(const Vector3d& position, double time) const = 0;

    /**
     * @brief Returns fluid density at given coordinates.
     *
     * @param[in] position Position at which fluid density is extracted.
     * @param[in] time Time of simulation.
     */
    virtual double get_fluid_density(const Vector3d& position, double time) const = 0;

    virtual Vector3d get_fluid_acceleration(const Vector3d& position, double time) const {
        return Vector3d(0.0, 0.0, 0.0);
    };
};

}  // namespace env
}  // namespace seahowl
