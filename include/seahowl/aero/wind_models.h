#pragma once

#include "seahowl/commons/numerics.h"

namespace seahowl {
namespace aero {

/**
 * @brief Base class for wind models
 */
class WindModel {
  public:
    /** @brief Air density. */
    double density = 1.225;

    /**
     * @brief Constructor.
     */
    WindModel();

    /**
     * @brief Returns wind velocity at given coordinates.
     *
     * @param[in] position Position at which wind velocity is extracted.
     * @param[in] time Time of simulation.
     */
    virtual Vector3d get_wind_velocity(const Vector3d& position, double time) const;

    /**
     * @brief Returns air density.
     */
    double get_density() const;
};

/**@brief Constant wind models */
class ConstantWind : public WindModel {
  public:
    /** @brief Constant wind velocity. */
    Vector3d wind_velocity;
    /** @brief Wind shear coefficient. */
    double shear_coefficient = 0.0;
    /** @brief Reference height (where constant velocity is defined). */
    double reference_height = 150.0;
    /** @brief Reference length (length of shear). */
    double reference_length = 240.0;
    /** @brief Direction of gravitational acceleration. */
    Vector3d direction_gravity;

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
    virtual Vector3d get_wind_velocity(const Vector3d& position, double time) const override;
};

/**@brief Wind ramp model */
class WindRamp : public WindModel {
  public:
    /** @brief Starting time of ramp. */
    double time_start = 0.0;
    /** @brief Ending time of ramp. */
    double time_stop = 1.0;
    /** @brief Wind velocity at beginning of ramp. */
    Vector3d wind_velocity_start;
    /** @brief Wind velocity at end of ramp. */
    Vector3d wind_velocity_stop;
    /** @brief Wind shear coefficient. */
    double shear_coefficient = 0.0;
    /** @brief Reference height (where constant velocity is defined). */
    double reference_height = 150.0;
    /** @brief Reference length (length of shear). */
    double reference_length = 240.0;
    /** @brief Direction of gravitational acceleration. */
    Vector3d direction_gravity;

    /**
     * @brief Constructor.
     */
    WindRamp();

    /**
     * @brief Sets wind velocity at beginning of ramp.
     *
     * @param[in] wind_velocity Wind velocity to use.
     */
    void set_wind_velocity_start(Vector3d velocity);

    /**
     * @brief Sets wind velocity at end of ramp.
     *
     * @param[in] wind_velocity Wind velocity to use.
     */
    void set_wind_velocity_stop(Vector3d velocity);

    /**
     * @brief Returns wind velocity at given coordinates.
     *
     * @param[in] position Position at which wind velocity is extracted.
     * @param[in] time Time of simulation.
     */
    virtual Vector3d get_wind_velocity(const Vector3d& position, double time) const override;
};

}  // namespace aero
}  // namespace seahowl
