#pragma once

#include "seahowl/env/env_model.h"

/**@brief Seahowl base namespace */
namespace seahowl {

class ComponentFluid {
  public:
    virtual ~ComponentFluid() = default;
    virtual void build() = 0;
    virtual void compute_env_loads(const env::EnvModel& env_model, double time) = 0;
    virtual void setup_environment(const env::EnvModel& env_model){};
};

}  // namespace seahowl
