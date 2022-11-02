#pragma once
#include <seahowl/elasto/elasto.h>

#include <chrono/fea/ChElementBeamEuler.h>
#include <chrono/fea/ChContactSurfaceNodeCloud.h>
#include <chrono/physics/ChMaterialSurfaceSMC.h>

namespace seahowl {
namespace elasto {

/**@brief Elastodynamic model for blade */
class MooringElasto : public ComponentElastoFEA {
  public:
    chrono::ChVector<double> fairlead_position = {0.0, 0.0, 0.0};
    chrono::ChVector<double> anchor_position = {0.0, 0.0, 0.0};
    double diameter = 0.0;
    double stiffness_axial = 0.0;
    double density = 0.0;
    double length = 0.0;
    std::shared_ptr<chrono::fea::ChContactSurfaceNodeCloud> contact_cloud;
    std::shared_ptr<chrono::ChMaterialSurfaceSMC> contact_material;

    MooringElasto();
    ~MooringElasto();

    void build();
    void build_elements_euler();
    void assemble(std::shared_ptr<chrono::fea::ChMesh> mesh);
};

}  // namespace elasto
}  // namespace seahowl
