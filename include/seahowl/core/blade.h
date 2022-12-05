#pragma once

#include <memory>
#include <vector>

#include <seahowl/core/reference_point.h>
#include <seahowl/core/utils.h>
#include <seahowl/elasto/blade_elasto.h>
#include <seahowl/aero/blade_aero.h>

namespace seahowl {
namespace core {

/**@brief Wind turbine blade base class

Pattern "mediator" for elasto and aero
*/
class Blade : public ComponentDynamic {
  public:
    std::shared_ptr<seahowl::elasto::BladeElasto> elasto;  ///< Elastodynamic element mesh
    std::shared_ptr<seahowl::aero::BladeAero> aero;        ///< Aerodynamic element mesh
    std::vector<seahowl::core::BladeReferencePoint>
        reference_points;  ///<@todo  Refactor: Only used for construction to pass to elasto and aero. Use a Builder
    std::vector<seahowl::core::DiscretizationPoint> mapping_aero2elasto_nodes;
    std::vector<seahowl::core::DiscretizationPoint> mapping_aero2elasto_elements;
    std::vector<seahowl::core::DiscretizationPoint> mapping_elasto2aero;

    Blade();
    ~Blade();

    void init(double time, double dt) override;
    void prestep(double time, double dt) override;
    void poststep(double time, double dt) override;
    void assemble(chrono::ChSystemSMC& system, std::shared_ptr<chrono::fea::ChMesh> mesh);
    void build();
    void set_discretization_elasto(std::vector<double> fractions);
    void set_discretization_aero(std::vector<double> fractions);
    void compute_mapping_aero2elasto();
    void compute_mapping_elasto2aero();

    /**@brief Compute aerodynamic loadings */
    void update_positions_aero();

    /**@brief Compute elastodynamic loadings */
    void update_loads_elasto();
};

}  // namespace core
}  // namespace seahowl
