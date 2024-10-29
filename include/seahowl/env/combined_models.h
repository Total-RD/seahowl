#pragma once

#include "seahowl/commons/numerics.h"
#include "seahowl/env/fluid_models.h"
#include "seahowl/env/wind_models.h"
#include "seahowl/env/wave_models.h"
#include "seahowl/env/soil_models.h"

#include <memory>

namespace seahowl {
namespace env {

class WaveWindModel : public FluidModel {
  public:
    std::shared_ptr<WindModel> wind_model;
    std::shared_ptr<WaveModel> wave_model;

    virtual double get_fluid_density(const Vector3d& position, double time) const override;

  protected:
    virtual Vector3d get_fluid_velocity_this(const Vector3d& position, double time) const override;
    virtual Vector3d get_fluid_acceleration_this(const Vector3d& position, double time) const override;
};

class FluidSoilModel : public FluidModel, public SoilModel {
  public:
    std::shared_ptr<FluidModel> fluid_model;
    std::shared_ptr<SoilModel> soil_model;

    virtual double get_fluid_density(const Vector3d& position, double time) const override;

    virtual bool is_in_soil(const Vector3d& position) const override;

    virtual Vector3d get_penetration_load(const EntityDynamic& entity,
                                          double contact_area,
                                          double entity_mass) const override;

  protected:
    virtual Vector3d get_fluid_velocity_this(const Vector3d& position, double time) const override;
    virtual Vector3d get_fluid_acceleration_this(const Vector3d& position, double time) const override;
};

}  // namespace env
}  // namespace seahowl
