#pragma once

#include <memory>
#include <vector>

#include <seahowl/core/reference_point.h>
#include <seahowl/core/utils.h>
#include <seahowl/elasto/blade_elasto.h>
#include <seahowl/aero/blade_aero.h>

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
    std::shared_ptr<seahowl::elasto::BladeElasto> elasto;
    /** @brief Aerodynamic model of the blade. */
    std::shared_ptr<seahowl::aero::BladeAero> aero;
    /** @brief List of reference points describing the blade properties along its longitudinal axis.
     * @todo  Refactor: Only used for construction to pass to elasto and aero. Use a Builder */
    std::vector<seahowl::core::BladeReferencePoint> reference_points;
    /** @brief Mapping of aero nodes into elasto domain. */
    std::vector<seahowl::core::DiscretizationPoint> mapping_aero2elasto_nodes;
    /** @brief Mapping of aero elements (central point of elements) into elasto domain. */
    std::vector<seahowl::core::DiscretizationPoint> mapping_aero2elasto_elements;
    /** @brief Mapping of elasto nodes into aero domain. */
    std::vector<seahowl::core::DiscretizationPoint> mapping_elasto2aero;

    /**
     * @brief Constructor.
     *
     * Instantiates elasto and aero blade components.
     */
    Blade();

    /**
     * @brief Initialize blade, called before starting the simulation.
     *
     * Runs the preset and poststep once to make elasto and aero components match.
     *
     * @param[in] time Time of the simulation (usually 0 at init).
     * @param[in] dt Time step length.
     */
    void init(double time, double dt) override;

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

    /**
     * @brief Builds the blade (aero and elasto part).
     *
     * Sets the nodes and elements for elasto and aero components of the blade, as well as the aero->elasto mapping and
     * elasto->aero mapping.
     */
    void build();

    /**
     * @brief Assembles the blade (elasto part).
     *
     * @param[out] mesh Mesh on which to add nodes and elements.
     */
    void assemble(std::shared_ptr<chrono::fea::ChMesh> mesh);

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
