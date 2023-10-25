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

    virtual Vector3d get_fluid_velocity(const Vector3d& position, double time) const override;

    virtual double get_fluid_density(const Vector3d& position, double time) const override;
};

class SoilWaveWindModel : public FluidModel, public SoilModel {
  public:
    std::shared_ptr<WindModel> wind_model;
    std::shared_ptr<WaveModel> wave_model;
    std::shared_ptr<SoilModel> soil_model;

    virtual Vector3d get_fluid_velocity(const Vector3d& position, double time) const override;

    virtual double get_fluid_density(const Vector3d& position, double time) const override;

    virtual Vector3d get_penetration_load(const EntityDynamic& entity,
                                          double contact_area,
                                          double entity_mass) const override;
};

}  // namespace env
}  // namespace seahowl
