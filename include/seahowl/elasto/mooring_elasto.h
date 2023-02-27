#pragma once
#include <seahowl/elasto/elasto.h>
#include <seahowl/utils.h>

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
    /** @brief Contact cloud of the mooring line. */
    std::shared_ptr<chrono::fea::ChContactSurfaceNodeCloud> contact_cloud;
    /** @brief Contact material of the mooring line. */
    std::shared_ptr<chrono::ChMaterialSurfaceSMC> contact_material;

    MooringElasto();
    ~MooringElasto();

    /**
     * @brief Builds the mooring (to call before assemble).
     */
    void build();

    /**
     * @brief Assembles the FEA component (adds all nodes and elements to mesh).
     *
     * @param[out] mesh Mesh on which to add nodes and elements.
     */
    void assemble(std::shared_ptr<chrono::fea::ChMesh> mesh);

  private:
    /**
     * @brief Builds the mooring with Euler-Bernoulli elements.
     */
    void build_elements_euler();
};

}  // namespace elasto
}  // namespace seahowl
