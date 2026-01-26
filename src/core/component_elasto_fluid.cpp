#include "seahowl/core/component_elasto_fluid.h"

namespace seahowl {
namespace core {

void ComponentElastoFluid::compute_mapping_elasto2fluid(const std::vector<double>& elasto_fractions,
                                                        const std::vector<double>& fluid_fractions) {
    mapping_elasto2fluid = get_indice_and_positions(elasto_fractions, fluid_fractions);
}

}  // namespace core
}  // namespace seahowl
