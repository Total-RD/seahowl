#pragma once

#include "seahowl/elasto/component_elasto.h"
#include "seahowl/commons/numerics.h"

namespace seahowl {
namespace elasto {

/**
 * @brief Mooring as an elastodynamic FEA component.
 *
 * Moorings are discretized into Euler-Bernoulli beam elements.
 */
class MooringElasto : public ComponentElastoFEA {
  public:
    /** @brief Position of the fairlead. */
    Vector3d fairlead_position = {0.0, 0.0, 0.0};
    /** @brief Position of the anchor. */
    Vector3d anchor_position = {0.0, 0.0, 0.0};
    /** @brief Position of the mooring line. */
    double diameter = 0.0;
    /** @brief Axial stiffness of the mooring line. */
    double stiffness_axial = 0.0;
    /** @brief Lineic density of the mooring line. */
    double density = 0.0;
    /** @brief Unstretched length of the mooring line. */
    double length = 0.0;
    /** @brief Drag coefficient of the mooring line. */
    double drag_coefficient = 0.5;
    /** @brief Added mass coefficient of the mooring line. */
    double added_mass_coefficient = 0.5;

    MooringElasto();

    /**
     * @brief Builds the mooring (to call before assemble).
     */
    void build();
    void build_nodes(const std::vector<ReferencePointElasto>& discretized_points);
    void compute_hydro_loads();

  private:
    /**
     * @brief Builds the mooring with ANCF cable elements.
     */
    void build_elements();
};

}  // namespace elasto
}  // namespace seahowl
