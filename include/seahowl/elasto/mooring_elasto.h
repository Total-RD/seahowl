#pragma once

#include "seahowl/elasto/component_elasto.h"
#include "seahowl/elasto/entities_elasto.h"
#include "seahowl/commons/numerics.h"
#include "seahowl/env/soil_models.h"

namespace seahowl {
namespace elasto {

class MooringElasto {
  public:
    BodyElasto& fairlead;
    BodyElasto& anchor;

    MooringElasto(BodyElasto& fairlead, BodyElasto& anchor);
};

/**
 * @brief Mooring as an elastodynamic FEA component.
 *
 * Moorings are discretized into Euler-Bernoulli beam elements.
 */
class MooringElastoFEA : public MooringElasto, public ComponentElastoFEA {
  public:
    /** @brief Link between line and fairlead. */
    std::unique_ptr<seahowl::elasto::Link> fairlead_link;
    /** @brief Link between line and anchor. */
    std::unique_ptr<seahowl::elasto::Link> anchor_link;
    /** @brief Position of the mooring line. */
    double diameter = 0.0;
    /** @brief Axial stiffness of the mooring line. */
    double stiffness_axial = 0.0;
    /** @brief Bending stiffness of the mooring line. */
    double stiffness_bending = 0.0;
    /** @brief Linear density of the mooring line. */
    double density_linear = 0.0;
    /** @brief Unstretched length of the mooring line. */
    double length = 0.0;
    /** @brief Drag coefficient (normal) of the mooring line. */
    double drag_coefficient_normal = 2.0;
    /** @brief Drag coefficient (tangential) of the mooring line. */
    double drag_coefficient_tangential = 1.15;
    /** @brief Added mass coefficient (normal) of the mooring line. */
    double added_mass_coefficient_normal = 1.0;
    /** @brief Added mass coefficient (tangential) of the mooring line. */
    double added_mass_coefficient_tangential = 1.0;

    MooringElastoFEA(BodyElasto& fairlead, BodyElasto& anchor);

    /**
     * @brief Builds the mooring (to call before assemble).
     */
    void build();

    void build_nodes(const std::vector<ReferencePointElasto>& discretized_points);

    virtual void assemble(SystemElasto& system) override;

    /**
     * @brief Computes hydro loads on cable.
     *
     * param[in] gravitational_acceleration Gravitational acceleration vector.
     * param[in] fluid_density Density of fluid.
     */
    void compute_hydro_loads(const Vector3d& gravitational_acceleration, double fluid_density);

    /**
     * @brief Computes seabed interaction loads on cable.
     *
     * param[in] seabed Seabed model.
     */
    void compute_seabed_loads(const seahowl::env::SoilModel& seabed);

  private:
    /**
     * @brief Builds the mooring with ANCF cable elements.
     */
    void build_elements();
};

}  // namespace elasto
}  // namespace seahowl
