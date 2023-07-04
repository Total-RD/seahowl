#pragma once

#include "seahowl/commons/numerics.h"
#include "seahowl/env/fluid_models.h"

namespace seahowl {
namespace env {

/**
 * @brief Base class for wind models
 */
class WindModel : public FluidModel {
  public:
    /** @brief Air density. */
    double density = 1.225;

    /**
     * @brief Returns air density.
     */
    virtual double get_fluid_density(const Vector3d& position, double time) const override;
};

class ShearedWind : public WindModel {
  public:
    /** @brief Wind shear coefficient. */
    double shear_coefficient = 0.0;
    /** @brief Reference height (where constant velocity is defined). */
    double reference_height = 150.0;
    /** @brief Reference length (length of shear). */
    double reference_length = 240.0;
    /** @brief Direction of gravitational acceleration. */
    Vector3d direction_gravity{0.0, 0.0, -1.0};
};

/**@brief Constant wind models */
class ConstantWind : public ShearedWind {
  public:
    /** @brief Wind velocity. */
    Vector3d wind_velocity;

    /**
     * @brief Constructor.
     */
    ConstantWind();

    /**
     * @brief Sets wind velocity.
     *
     * @param[in] wind_velocity Wind velocity to use as constant.
     */
    void set_wind_velocity(Vector3d velocity);

    /**
     * @brief Returns wind velocity at given coordinates.
     *
     * @param[in] position Position at which wind velocity is extracted.
     * @param[in] time Time of simulation.
     */
    virtual Vector3d get_fluid_velocity(const Vector3d& position, double time) const override;
};

/**@brief Wind ramp model */
class WindRamp : public ShearedWind {
  public:
    /** @brief Starting time of ramp. */
    double time_start = 0.0;
    /** @brief Ending time of ramp. */
    double time_end = 0.0;
    /** @brief Wind velocity at startning of ramp. */
    Vector3d wind_velocity_start{0.0, 0.0, 0.0};
    /** @brief Wind velocity at end of ramp. */
    Vector3d wind_velocity_end{0.0, 0.0, 0.0};

    /**
     * @brief Constructor.
     */
    WindRamp();

    /**
     * @brief Sets wind ramp.
     *
     * @param[in] velocity_start Wind velocity at starting of ramp.
     * @param[in] time_start Time at which the ramp starts.
     * @param[in] velocity_end Wind velocity at end of ramp.
     * @param[in] time_end Time at which the ramp ends.
     */
    void set_wind_ramp(const Vector3d& velocity_start,
                       double time_start,
                       const Vector3d& velocity_end,
                       double time_end);

    /**
     * @brief Returns wind velocity at given coordinates.
     *
     * @param[in] position Position at which wind velocity is extracted.
     * @param[in] time Time of simulation.
     */
    virtual Vector3d get_fluid_velocity(const Vector3d& position, double time) const override;
};

}  // namespace env
}  // namespace seahowl
