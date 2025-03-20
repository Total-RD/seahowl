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
    double ramp_start = 0.0;
    double ramp_end = 0.0;

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

  private:
    /**
     * @brief Applies ramp to the given vector.
     */
    void apply_ramp(double& time, Vector3d& res) const;

  protected:
    virtual Vector3d get_acceleration_this(const Vector3d& position, double time) const = 0;
    virtual Vector3d get_velocity_this(const Vector3d& position, double time) const = 0;
    virtual double get_density_this(const Vector3d& position, double time) const = 0;
};

}  // namespace env
}  // namespace seahowl
