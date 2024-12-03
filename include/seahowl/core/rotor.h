#pragma once

#include "seahowl/core/component.h"
#include "seahowl/commons/numerics.h"

#include <vector>
#include <memory>

// forward declarations
namespace seahowl {
namespace core {
class Blade;
}  // namespace core
namespace elasto {
class RotorNacelleAssemblyElasto;
}  // namespace elasto
namespace aero {
class RotorNacelleAssemblyAero;
}  // namespace aero
}  // namespace seahowl

namespace seahowl {
namespace core {

/**
 * @brief Rotor-Nacelle Assembly (RNA) of wind turbine, with both elasto and aero components.
 *
 * This class acts as a "mediator" between the elasto and aero components.
 * The aero position of the RNA is updated using the elasto position.
 */
class RotorNacelleAssembly : public ComponentDynamic {
  public:
    /** @brief Elastodynamic model of the RNA. */
    seahowl::elasto::RotorNacelleAssemblyElasto& elasto;
    /** @brief Aerodynamic model of the RNA. */
    seahowl::aero::RotorNacelleAssemblyAero& aero;
    /** @brief Blades of the turbine. */
    std::vector<std::shared_ptr<seahowl::core::Blade>> blades;

    /**
     * @brief Instantiates rotor for communication between elasto and aero components.
     *
     * @param[in] elasto Elastodynamic RNA model.
     * @param[in] aero Aerodynamic RNA model.
     */
    RotorNacelleAssembly(seahowl::elasto::RotorNacelleAssemblyElasto& elasto,
                         seahowl::aero::RotorNacelleAssemblyAero& aero);

    /**
     * @brief Prestep for RNA, called before elastodynamic stepping.
     *
     * Calls prestep for each blade.
     *
     * @param[in] time Time of the simulation.
     * @param[in] dt Time step length.
     */
    void prestep(double time, double dt) override;

    /**
     * @brief Poststep for RNA, called after elastodynamic stepping.
     *
     * Calls poststep for each blade.
     *
     * @param[in] time Time of the simulation.
     * @param[in] dt Time step length.
     */
    void poststep(double time, double dt) override;

    void apply_fluid_model(seahowl::env::FluidModel& fluid_model, double time) override;

    /**
     * @brief Updates aero positions, rotations, velocities and accelerations from elasto component of the RNA.
     */
    void update_positions_aero();

    /**
     * @brief Builds the RNA and blades associated to it.
     */
    void build() override;

    /**
     * @brief Returns yaw error.
     *
     * The yaw error is defined as the angle bteween the rotor disk normal vector to the rotor-disk-averaged relative
     * wind velocity, both projected on global X-Y plane.
     */
    double get_yaw_error() const;

  private:
    /**
     * @brief Initialize RNA, called before starting the simulation.
     *
     * Runs preset and poststep once to make elasto and aero components match.
     *
     * @param[in] time Time of the simulation (usually 0 at init).
     * @param[in] dt Time step length.
     */
    void initialize_this(double time, double dt) override;
};

}  // namespace core
}  // namespace seahowl
