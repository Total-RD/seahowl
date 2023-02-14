#pragma once
#include <seahowl/core/turbine.h>
#include <seahowl/aero/wind_models.h>
#include <seahowl/servo/controller.h>
#include <seahowl/servo/controller_discon.h>
#include <chrono/physics/ChBody.h>

namespace seahowl {
namespace core {

class System {
  public:
    Turbine turbine;
    std::shared_ptr<seahowl::aero::WindModel> wind_model;

    System(Turbine turbine);
    ~System();

    virtual void init(double time, double dt);
    virtual void prestep(double time, double dt);
    virtual void poststep(double time, double dt);
};
}  // namespace core
}  // namespace seahowl