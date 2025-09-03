#pragma once

#include "seahowl/commons/component_fluid.h"

namespace seahowl {

/**@brief Hydrodynamic module */
namespace hydro {

class FoundationFluid : public virtual ComponentFluid {
  public:
    /**
     * @brief Virtual destructor.
     */
    virtual ~FoundationFluid() = default;
};

}  // namespace hydro
}  // namespace seahowl
