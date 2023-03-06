#pragma once

#include <seahowl/elasto/component_elasto.h>
#include <seahowl/commons.h>

#include <chrono/fea/ChElementBeamEuler.h>
#include <chrono/fea/ChContactSurfaceNodeCloud.h>
#include <chrono/physics/ChMaterialSurfaceSMC.h>

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

    MooringElasto();

    /**
     * @brief Builds the mooring (to call before assemble).
     */
    void build();

  private:
    /**
     * @brief Builds the mooring with Euler-Bernoulli elements.
     */
    void build_elements_euler();
};

}  // namespace elasto
}  // namespace seahowl
