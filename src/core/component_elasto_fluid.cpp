#include "seahowl/core/component_elasto_fluid.h"

#include "seahowl/elasto/component_elasto.h"

namespace seahowl {
namespace core {

ComponentElastoFluid::ComponentElastoFluid(const std::shared_ptr<seahowl::elasto::ComponentElasto> elasto,
                                           const std::shared_ptr<seahowl::fluid::ComponentFluid> fluid)
    : ComponentDynamic(elasto, fluid), elasto(*elasto), fluid(*fluid) {}

void ComponentElastoFluid::compute_mapping_elasto2fluid() {
    // Map elasto nodes to fluid domain
    mapping_elasto2fluid_nodes =
        get_indice_and_positions(elasto.discretization_fractions, fluid.discretization_fractions);
}

void ComponentElastoFluid::compute_mapping_fluid2elasto() {
    // Map fluid nodes to elasto elements
    mapping_fluid2elasto_nodes =
        get_indice_and_positions(fluid.discretization_fractions, elasto.discretization_fractions);

    // Compute element fractions as midpoints between node fractions
    std::vector<double> fluid_element_fractions;
    for (size_t ii = 0; ii < fluid.discretization_fractions.size() - 1; ii++) {
        auto element_fraction = 0.5 * (fluid.discretization_fractions[ii] + fluid.discretization_fractions[ii + 1]);
        fluid_element_fractions.push_back(element_fraction);
    }

    // Map fluid elements to elasto elements
    mapping_fluid2elasto_elements = get_indice_and_positions(fluid_element_fractions, elasto.discretization_fractions);
}

}  // namespace core
}  // namespace seahowl
