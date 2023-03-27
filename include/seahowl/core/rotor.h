#pragma once

#include <seahowl/core/blade.h>
#include <seahowl/elasto/rotor_elasto.h>
#include <seahowl/aero/rotor_aero.h>

namespace seahowl {
namespace core {

/**
 * @brief Rotor-Nacelle Assembly (RNA) of wind turbine, with both elasto and aero components.
 *
 * This class acts as a "mediator" between the elasto and aero components.
 * The aero position of the RNA is updated using the elasto position.
 */
class Rotor : public ComponentDynamic {
  public:
    /** @brief Elastodynamic model of the RNA. */
    seahowl::elasto::RotorElasto& elasto;
    /** @brief Aerodynamic model of the RNA. */
    seahowl::aero::RotorAero& aero;
    /** @brief Blades of the turbine. */
    std::vector<std::shared_ptr<seahowl::core::Blade>> blades;

    /**
     * @brief Instantiates rotor for communication between elasto and aero components.
     *
     * @param[in] elasto Elastodynamic RNA model.
     * @param[in] aero Aerodynamic RNA model.
     */
    Rotor(seahowl::elasto::RotorElasto& elasto, seahowl::aero::RotorAero& aero);

    /**
     * @brief Initialize RNA, called before starting the simulation.
     *
     * Runs preset and poststep once to make elasto and aero components match.
     *
     * @param[in] time Time of the simulation (usually 0 at init).
     * @param[in] dt Time step length.
     */
    void init(double time, double dt) override;

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

    /**
     * @brief Updates aero positions, rotations, velocities and accelerations from elasto component of the RNA.
     */
    void update_positions_aero();

    /**
     * @brief Assembles the RNA (elasto part).
     *
     * @param[out] system System on which to add bodies and links.
     * @param[out] mesh Mesh on which to add nodes and elements.
     */
    void assemble(std::shared_ptr<seahowl::elasto::SystemElasto> system,
                  std::shared_ptr<seahowl::elasto::MeshElasto> mesh);

    /**
     * @brief Builds the RNA and blades associated to it.
     */
    void build();
};

}  // namespace core
}  // namespace seahowl
