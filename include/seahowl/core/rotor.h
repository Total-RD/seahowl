#pragma once

#include <seahowl/core/blade.h>
#include <seahowl/elasto/rotor_elasto.h>
#include <seahowl/aero/rotor_aero.h>

namespace seahowl {
namespace core {

/**@brief Wind turbine rotor: Hub + blades

@todo Rotor should be composed of blades  + hub
*/
class Rotor : public ComponentDynamic {
  public:
    seahowl::elasto::RotorElasto elasto;                        ///< Elastodynamic model
    seahowl::aero::RotorAero aero;                              ///< Aerodynamic model
    std::vector<std::shared_ptr<seahowl::core::Blade>> blades;  ///< Blades @todo connect bladeaero through blade ?

    Rotor();
    ~Rotor();

    void init(double time, double dt) override;
    void prestep(double time, double dt) override;
    void poststep(double time, double dt) override;
    void update_positions_aero();
    void assemble(chrono::ChSystemSMC& system);
    void build(std::vector<std::shared_ptr<seahowl::core::Blade>> blades);
};

}  // namespace core
}  // namespace seahowl
