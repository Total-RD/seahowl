#pragma once

#include "seahowl/env/env_model.h"

/**@brief Seahowl base namespace */
namespace seahowl {

class ComponentFluid {
  public:
    virtual void build() = 0;
    virtual void compute_fluid_loads(const env::EnvModel& fluid_model, double time) = 0;
};

}  // namespace seahowl
