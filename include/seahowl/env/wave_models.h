#pragma once

#include "seahowl/commons/numerics.h"
#include "seahowl/env/fluid_models.h"

namespace seahowl {
namespace env {

/**
 * @brief Base class for wave models
 */
class WaveModel : public FluidModel {
  public:
    /** @brief Water density. */
    double density = 1025;
    /** @brief Mean water level. */
    double mean_water_level = 0.0;
    /** @brief Free surface normal. */
    Vector3d surface_normal{0.0, 0.0, 1.0};
    /** @brief Water depth. */
    double water_depth = 0.0;

    /**
     * @brief Returns whether position at time t in inside water.
     *
     * @param[in] position Position at which fluid velocity is extracted.
     * @param[in] time Time of simulation.
     */
    bool is_in_water(const Vector3d& position, double time) const;

    /**
     * @brief Returns water level.
     */
    virtual double get_water_level(const Vector3d& position, double time) const = 0;
};

/**
 * @brief Model for still water (no wave, no current).
 */
class StillWater : public WaveModel {
  public:
    /**
     * @brief Constructor.
     */
    StillWater();

    virtual double get_fluid_density(const Vector3d& position, double time) const override;

    virtual double get_water_level(const Vector3d& position, double time) const override;

  protected:
    virtual Vector3d get_fluid_velocity_this(const Vector3d& position, double time) const override;
    virtual Vector3d get_fluid_acceleration_this(const Vector3d& position, double time) const override;
};

/**
 * @brief Model for constant current with only horizontal velocity components.
 */
class CurrentConstant : public StillWater {
  public:
    /** @brief Horizontal velocity of fluid at the free surface. */
    double velocity_surface = 0.0;
    /** @brief Horizontal velocity of fluid at the seabed. */
    double velocity_seabed = 0.0;
    /** @brief Current direction. */
    Vector3d direction{1.0, 0.0, 0.0};
    /** @brief Power factor for for power law exponent (1/power_factor). */
    double power_factor = 7;

    /**
     * @brief Constructor.
     */
    CurrentConstant();

  protected:
    virtual Vector3d get_fluid_velocity_this(const Vector3d& position, double time) const override;
    virtual Vector3d get_fluid_acceleration_this(const Vector3d& position, double time) const override;
};

}  // namespace env
}  // namespace seahowl
