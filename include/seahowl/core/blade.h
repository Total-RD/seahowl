#pragma once

#include "seahowl/core/component.h"

#include <memory>
#include <vector>

// forward declarations
namespace seahowl {
struct DiscretizationPoint;
namespace core {
class Blade;
}  // namespace core
namespace elasto {
class BladeElasto;
}  // namespace elasto
namespace aero {
class BladeAero;
}  // namespace aero
}  // namespace seahowl

namespace seahowl {
namespace core {

/**
 * @brief Blade of wind turbine, with both elasto and aero components.
 *
 * This class acts as a "mediator" between the elasto and aero components.
 * Mappings between elasto nodes to aero domain and aero nodes to elasto domain are used to ensure communication between
 * the aero and elasto components. The aero loads are communicated to the elasto component in the prestep, while the
 * aero positions are updated using the elasto positions in the poststep.
 */
class Blade : public ComponentDynamic {
  public:
    /** @brief Elastodynamic model of the blade. */
    seahowl::elasto::BladeElasto& elasto;
    /** @brief Aerodynamic model of the blade. */
    seahowl::aero::BladeAero& aero;
    /** @brief Mapping of aero nodes into elasto domain. */
    std::vector<seahowl::DiscretizationPoint> mapping_aero2elasto_nodes;
    /** @brief Mapping of aero elements (central point of elements) into elasto domain. */
    std::vector<seahowl::DiscretizationPoint> mapping_aero2elasto_elements;
    /** @brief Mapping of elasto nodes into aero domain. */
    std::vector<seahowl::DiscretizationPoint> mapping_elasto2aero;

    /**
     * @brief Instantiates blade for communication between elasto and aero components.
     *
     * @param[in] elasto Elastodynamic blade model.
     * @param[in] aero Aerodynamic blade model.
     */
    Blade(seahowl::elasto::BladeElasto& elasto, seahowl::aero::BladeAero& aero);

    /**
     * @brief Prestep for blade, called before elastodynamic stepping.
     *
     * Updates aero loads on elasto component.
     *
     * @param[in] time Time of the simulation.
     * @param[in] dt Time step length.
     */
    void prestep(double time, double dt) override;

    /**
     * @brief Poststep for blade, called after elastodynamic stepping.
     *
     * Updates aero positions from elasto component.
     *
     * @param[in] time Absolute time of the simulation.
     * @param[in] dt Time step length.
     */
    void poststep(double time, double dt) override;

    void apply_env_model(seahowl::env::EnvModel& env_model, double time) override;

    /**
     * @brief Builds the blade (aero and elasto part).
     *
     * Sets the nodes and elements for elasto and aero components of the blade, as well as the aero->elasto mapping and
     * elasto->aero mapping.
     */
    virtual void build();

    /**
     * @brief Applies pitch increment to the blade (i.e. rotates the blade around its longitudinal axis).
     *
     * @param pitch_increment Pitch increment value (in radians).
     */
    void apply_pitch_increment(double pitch_increment);

    /**
     * @brief Sets the discretization fractions to use when building the elasto part of the blade.
     *
     * @param[in] fractions Normalized discretization fractions within [0, 1].
     */
    void set_discretization_elasto(std::vector<double> fractions);

    /**
     * @brief Sets the discretization fractions to use when building the aero part of the blade.
     *
     * @param[in] fractions Normalized discretization fractions within [0, 1].
     */
    void set_discretization_aero(std::vector<double> fractions);

    /**
     * @brief Updates aero positions, rotations, velocities and accelerations from elasto component of the blade.
     */
    void update_positions_aero();

    /**
     * @brief Accumulates aero loads to the elasto component of the blade.
     */
    void update_loads_elasto();

  private:
    /**
     * @brief Initialize blade, called before starting the simulation.
     *
     * Runs the preset and poststep once to make elasto and aero components match.
     *
     * @param[in] time Time of the simulation (usually 0 at init).
     * @param[in] dt Time step length.
     */
    void initialize_this(double time, double dt) override;

    /**
     * @brief Computes the aero->elasto mapping that is used when accumulating aero loads on elasto component.
     */
    void compute_mapping_aero2elasto();

    /**
     * @brief Computes the elasto->aero mapping.
     */
    void compute_mapping_elasto2aero();
};

}  // namespace core
}  // namespace seahowl
