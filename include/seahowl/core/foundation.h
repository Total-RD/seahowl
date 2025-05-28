#pragma once

#include "seahowl/core/component.h"

namespace seahowl {
namespace core {

class Foundation : public virtual ComponentDynamic {
  public:
    Foundation(const std::shared_ptr<seahowl::elasto::ComponentElasto>& elasto_,
               const std::shared_ptr<seahowl::ComponentFluid>& fluid_)
        : ComponentDynamic(elasto_, fluid_) {}

    virtual ~Foundation() = default;

};

}  // namespace core
}  // namespace seahowl
