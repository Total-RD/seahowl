#pragma once
#include <seahowl/elasto/elasto.h>

#include <chrono/fea/ChElementBeamEuler.h>

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

    MooringElasto();
    ~MooringElasto();

    void build();
    void build_elements_euler();
};

}  // namespace elasto
}  // namespace seahowl
