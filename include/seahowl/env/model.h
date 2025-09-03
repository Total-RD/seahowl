#pragma once

#include "seahowl/commons/numerics.h"

namespace seahowl {
namespace env {

/**
 * @brief Base class for models
 */
class Model {
  public:
    /**
     * @brief Returns true is the placement of the model (false otherwise).
     *
     * @param[in] position Position to assess whether inside model or not.
     * @param[in] time Time of simulation.
     */
    virtual bool is_inside(const Vector3d& position, double time = 0.0) const = 0;
    /**
     * @brief Virtual destructor.
     */
    virtual ~Model() = default;
};

}  // namespace env
}  // namespace seahowl
