#pragma once

#include "seahowl/env/fluid_models.h"

/**@brief Seahowl base namespace */
namespace seahowl {

class ComponentFluid {
  public:
    virtual void compute_fluid_loads(const env::FluidModel& fluid_model, double time) = 0;
};

}  // namespace seahowl
