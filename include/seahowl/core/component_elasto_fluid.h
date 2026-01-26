#pragma once

#include "seahowl/core/component.h"
#include "seahowl/commons/utils.h"

#include <vector>

namespace seahowl {
namespace core {

/**
 * @brief Base class for components coupling elastodynamic and fluid domains.
 *
 * Provides common mapping infrastructure for components that mediate between
 * an elastic structural model and a fluid model (aero or hydro).
 */
class ComponentElastoFluid : public virtual ComponentDynamic {
  protected:
    /** @brief Mapping of fluid nodes into elasto domain. */
    std::vector<seahowl::DiscretizationPoint> mapping_fluid2elasto_nodes;
    /** @brief Mapping of fluid elements (central point of elements) into elasto domain. */
    std::vector<seahowl::DiscretizationPoint> mapping_fluid2elasto_elements;
    /** @brief Mapping of elasto nodes into fluid domain. */
    std::vector<seahowl::DiscretizationPoint> mapping_elasto2fluid;

    /**
     * @brief Computes mapping from elasto discretization to fluid discretization.
     * @param[in] elasto_fractions Discretization fractions of the elastic component.
     * @param[in] fluid_fractions Discretization fractions of the fluid component.
     */
    void compute_mapping_elasto2fluid(const std::vector<double>& elasto_fractions,
                                      const std::vector<double>& fluid_fractions);
};

}  // namespace core
}  // namespace seahowl
