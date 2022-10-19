#pragma once

#include <memory>
#include <vector>

#include <seahowl/elasto/tower_elasto.h>
#include <seahowl/aero/tower_aero.h>
#include <seahowl/core/reference_point.h>

namespace chrono {
namespace fea {
class ChMesh;
}
}  // namespace chrono

namespace seahowl {
namespace core {

class Tower : public ComponentDynamic {
  public:
    seahowl::elasto::TowerElasto elasto;  ///< Elastodynamic element mesh
    seahowl::aero::TowerAero aero;        ///< Aerodynamic element mesh
    std::vector<seahowl::core::TowerReferencePoint>
        reference_points;  ///<@todo  Refactor: Only used for construction to pass to elasto and aero. Use a Builder
    std::vector<seahowl::core::DiscretizationPoint> mapping_aero2elasto;
    std::vector<seahowl::core::DiscretizationPoint> mapping_elasto2aero;

    Tower();
    ~Tower();

    void init(double time, double dt) override;
    void prestep(double time, double dt) override;
    void poststep(double time, double dt) override;
    void assemble(std::shared_ptr<chrono::fea::ChMesh> mesh);
    void build();
    void set_discretization_elasto(std::vector<double> fractions);
    void set_discretization_aero(std::vector<double> fractions);
    void compute_mapping_aero2elasto();
    void compute_mapping_elasto2aero();

    /**@brief Updates positions for aero elements */
    void update_positions_aero();

    /**@brief Applies elastodynamic loadings */
    void update_loads_elasto();
};

}  // namespace core
}  // namespace seahowl
