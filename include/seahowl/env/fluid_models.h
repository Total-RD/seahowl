#pragma once

#include "seahowl/commons/numerics.h"
#include "seahowl/env/model.h"

namespace seahowl {
namespace env {

/**
 * @brief Base class for fluid models
 */
class FluidModel : public Model {
  public:
    /**
     * @brief Returns fluid density at given coordinates.
     *
     * @param[in] position Position at which fluid density is extracted.
     * @param[in] time Time of simulation.
     */
    virtual double get_density(const Vector3d& position, double time) const;
    /**
     * @brief Returns fluid density at given coordinates where the position is inside the fluid.
     * @param[in] position Position at which fluid density is extracted.
     * @param[in] time Time of simulation.
     */
    virtual double get_density_inside(const Vector3d& position, double time) const;
    /**
     * @brief Returns fluid velocity at given coordinates.
     *
     * @param[in] position Position at which fluid velocity is extracted.
     * @param[in] time Time of simulation.
     */
    Vector3d get_velocity(const Vector3d& position, double time) const;

    /**
     * @brief Returns fluid velocity at given coordinates where the position is inside the fluid.
     *
     * @param[in] position Position at which fluid velocity is extracted.
     * @param[in] time Time of simulation.
     */
    Vector3d get_velocity_inside(const Vector3d& position, double time) const;

    /**
     * @brief Returns fluid acceleration at given coordinates.
     *
     * @param[in] position Position at which fluid acceleration is extracted.
     * @param[in] time Time of simulation.
     */
    virtual Vector3d get_acceleration(const Vector3d& position, double time) const;

    /**
     * @brief Returns fluid acceleration at given coordinates where the position is inside the fluid..
     *
     * @param[in] position Position at which fluid acceleration is extracted.
     * @param[in] time Time of simulation.
     */
    virtual Vector3d get_acceleration_inside(const Vector3d& position, double time) const;
    /**
     * @brief Set ramp time.
     * @param[in] start_time Start time of the ramp.
     * @param[in] end_time End time of the ramp.
     */
    void set_ramp(double start_time, double end_time) {
        ramp_start = start_time;
        ramp_end = end_time;
    }

  private:
    double ramp_start = 0.0;
    double ramp_end = 0.0;
    /**
     * @brief Applies ramp factor to the given vector based on current time.
     *
     * @param[in] time Current simulation time.
     * @param[in,out] res Vector to which the ramp factor is applied.
     */
    void apply_ramp(double time, Vector3d& res) const;

  protected:
    /**
     * @brief Returns fluid acceleration at given coordinates (implementation).
     *
     * @param[in] position Position at which fluid acceleration is computed.
     * @param[in] time Time of simulation.
     * @return Fluid acceleration vector.
     */
    virtual Vector3d get_acceleration_this(const Vector3d& position, double time) const = 0;

    /**
     * @brief Returns fluid velocity at given coordinates (implementation).
     *
     * @param[in] position Position at which fluid velocity is computed.
     * @param[in] time Time of simulation.
     * @return Fluid velocity vector.
     */
    virtual Vector3d get_velocity_this(const Vector3d& position, double time) const = 0;

    /**
     * @brief Returns fluid density at given coordinates (implementation).
     *
     * @param[in] position Position at which fluid density is computed.
     * @param[in] time Time of simulation.
     * @return Fluid density value.
     */
    virtual double get_density_this(const Vector3d& position, double time) const = 0;
};

}  // namespace env
}  // namespace seahowl
