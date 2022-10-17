#pragma once

#include <seahowl/elasto/elasto.h>

#include <chrono/fea/ChElementBeamTaperedTimoshenko.h>

namespace chrono {
namespace fea {
class ChMesh;
}
}  // namespace chrono

namespace seahowl {
namespace elasto {

/**@brief Wind turbine tower elastodynamic model

Implemented as Finite Element Beams
*/
class TowerElasto : public ComponentElastoFEA {
  public:
    std::vector<TowerReferencePointElasto> reference_points;
    std::vector<TowerReferencePointElasto> discretized_points;
    double height;
    double base_height;

    TowerElasto();
    ~TowerElasto();

    void build();
    void build_elements_tapered_timoshenko();

    virtual void set_damping_coefficients(double axial, double edge, double flap, double torsion);
};

}  // namespace elasto
}  // namespace seahowl
