#pragma once

#include "seahowl/commons/numerics.h"

namespace seahowl {
namespace env {

/**
 * @brief Base class for fluid models
 */
class FluidModel {
  public:
    double ramp_start = 0.0;
    double ramp_end = 0.0;

    /**
     * @brief Returns fluid velocity at given coordinates.
     *
     * @param[in] position Position at which fluid velocity is extracted.
     * @param[in] time Time of simulation.
     */
    Vector3d get_fluid_velocity(const Vector3d& position, double time) const {
        Vector3d velocity = get_fluid_velocity_this(position, time);
        if (time < ramp_end) {
            if (time > ramp_start) {
                double ramp_fraction = time / (ramp_end - ramp_start);
                velocity *= ramp_fraction;
            } else {
                velocity *= 0.0;
            }
        }
        return velocity;
    };

    /**
     * @brief Returns fluid density at given coordinates.
     *
     * @param[in] position Position at which fluid density is extracted.
     * @param[in] time Time of simulation.
     */
    virtual double get_fluid_density(const Vector3d& position, double time) const = 0;

    virtual Vector3d get_fluid_acceleration(const Vector3d& position, double time) const {
        Vector3d acceleration = get_fluid_acceleration_this(position, time);
        if (time < ramp_end) {
            if (time > ramp_start) {
                double ramp_fraction = time / (ramp_end - ramp_start);
                acceleration *= ramp_fraction;
            } else {
                acceleration *= 0.0;
            }
        }
        return acceleration;
    };

  protected:
    virtual Vector3d get_fluid_acceleration_this(const Vector3d& position, double time) const {
        return Vector3d(0.0, 0.0, 0.0);
    };
    virtual Vector3d get_fluid_velocity_this(const Vector3d& position, double time) const = 0;
};

}  // namespace env
}  // namespace seahowl
