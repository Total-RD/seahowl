#pragma once

#include "seahowl/commons/numerics.h"
#include "seahowl/elasto/component_elasto.h"
#include "seahowl/elasto/entities_elasto.h"

namespace seahowl {
namespace elasto {

class FoundationElasto : public ComponentElasto {
  public:
    virtual void link_to_entity(const Entity& entity) = 0;
    virtual void prestep(double time, double dt) = 0;
};

}  // namespace elasto
}  // namespace seahowl
