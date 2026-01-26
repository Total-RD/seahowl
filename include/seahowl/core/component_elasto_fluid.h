#pragma once

#include "seahowl/core/component.h"
#include "seahowl/fluid/component_fluid.h"
#include "seahowl/commons/utils.h"

#include <memory>
#include <vector>

// forward declarations
namespace seahowl {
namespace elasto {
class ComponentElastoDiscretized;
}  // namespace elasto
}  // namespace seahowl

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
    /** @brief Reference to the discretized elasto component. */
    seahowl::elasto::ComponentElastoDiscretized& elasto_discretized;
    /** @brief Reference to the fluid component. */
    seahowl::fluid::ComponentFluid& fluid;

    /** @brief Mapping of fluid nodes into elasto domain. */
    std::vector<seahowl::DiscretizationPoint> mapping_fluid2elasto_nodes;
    /** @brief Mapping of fluid elements (central point of elements) into elasto domain. */
    std::vector<seahowl::DiscretizationPoint> mapping_fluid2elasto_elements;
    /** @brief Mapping of elasto nodes into fluid domain. */
    std::vector<seahowl::DiscretizationPoint> mapping_elasto2fluid_nodes;

    /**
     * @brief Constructor.
     * @param[in] elasto Shared pointer to the discretized elasto component.
     * @param[in] fluid Shared pointer to the fluid component.
     */
    ComponentElastoFluid(const std::shared_ptr<seahowl::elasto::ComponentElastoDiscretized> elasto,
                         const std::shared_ptr<seahowl::fluid::ComponentFluid> fluid);

    /**
     * @brief Computes mapping from elasto discretization to fluid discretization.
     */
    void compute_mapping_elasto2fluid();

    /**
     * @brief Computes mapping from fluid to elasto discretization.
     * Element fractions are computed as midpoints between node fractions.
     */
    void compute_mapping_fluid2elasto();
};

}  // namespace core
}  // namespace seahowl
