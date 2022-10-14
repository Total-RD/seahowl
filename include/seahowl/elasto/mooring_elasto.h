#pragma once
#include <seahowl/elasto/elasto.h>

namespace chrono {
class ChSystemSMC;
namespace fea {
class ChMesh;
class ChNodeFEAxyzrot;
class ChElementBeamTaperedTimoshenko;
}  // namespace fea
}  // namespace chrono

namespace seahowl {
namespace elasto {

/**@brief Elastodynamic model for blade */
class MooringElasto : public ElastoFEAComponent {
  public:
    double diameter;
    double young_modulus;
    double density;

    MooringElasto();
    ~MooringElasto();

    void build();
    void build_nodes();
    void build_elements_euler();
};

}  // namespace elasto
}  // namespace seahowl
