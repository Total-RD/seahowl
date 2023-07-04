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

    /**
     * @brief Returns whether position at time t in inside water.
     *
     * @param[in] position Position at which fluid velocity is extracted.
     * @param[in] time Time of simulation.
     */
    virtual bool is_in_water(const Vector3d& position, double time) const = 0;
};

class StillWater : public WaveModel {
  public:
    /** @brief Mean water level. */
    double mean_water_level = 0.0;

    /**
     * @brief Constructor.
     */
    StillWater();

    virtual Vector3d get_fluid_velocity(const Vector3d& position, double time) const override;

    virtual double get_fluid_density(const Vector3d& position, double time) const override;

    virtual bool is_in_water(const Vector3d& position, double time) const override;
};

}  // namespace env
}  // namespace seahowl
