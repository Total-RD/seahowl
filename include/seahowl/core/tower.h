#pragma once

#include <memory>
#include <vector>

#include <seahowl/elasto/entities_elasto.h>
#include <seahowl/elasto/tower_elasto.h>
#include <seahowl/aero/tower_aero.h>
#include <seahowl/core/reference_point.h>
#include <seahowl/core/component.h>

namespace seahowl {
namespace core {

/**
 * @brief Tower of wind turbine, with both elasto and aero components.
 *
 * This class acts as a "mediator" between the elasto and aero components.
 * Mappings between elasto nodes to aero domain and aero nodes to elasto domain are used to ensure communication between
 * the aero and elasto components. The aero loads are communicated to the elasto component in the prestep, while the
 * aero positions are updated using the elasto positions in the poststep.
 */
class Tower : public ComponentDynamic {
  public:
    /** @brief Elastodynamic model of the tower. */
    seahowl::elasto::TowerElasto& elasto;
    /** @brief Aerodynamic model of the tower. */
    seahowl::aero::TowerAero& aero;
    /** @brief List of reference points describing the tower properties along its longitudinal axis.
     * @todo  Refactor: Only used for construction to pass to elasto and aero. Use a Builder */
    std::vector<seahowl::core::TowerReferencePoint> reference_points;
    /** @brief Mapping of aero elements into elasto domain. */
    std::vector<seahowl::DiscretizationPoint> mapping_aero2elasto;
    /** @brief Mapping of elasto nodes into aero domain. */
    std::vector<seahowl::DiscretizationPoint> mapping_elasto2aero;

    /**
     * @brief Instantiates tower for communication between elasto and aero components.
     *
     * @param[in] elasto Elastodynamic tower model.
     * @param[in] aero Aerodynamic tower model.
     */
    Tower(seahowl::elasto::TowerElasto& elasto, seahowl::aero::TowerAero& aero);

    /**
     * @brief Initialize tower, called before starting the simulation.
     *
     * Runs the preset and poststep once to make elasto and aero components match.
     *
     * @param[in] time Time of the simulation (usually 0 at init).
     * @param[in] dt Time step length.
     */
    void initialize(double time, double dt) override;

    /**
     * @brief Prestep for tower, called before elastodynamic stepping.
     *
     * Updates aero loads on elasto component.
     *
     * @param[in] time Time of the simulation.
     * @param[in] dt Time step length.
     */
    void prestep(double time, double dt) override;

    /**
     * @brief Poststep for tower, called afetr elastodynamic stepping.
     *
     * Updates aero positions from elasto component.
     *
     * @param[in] time Absolute time of the simulation.
     * @param[in] dt Time step length.
     */
    void poststep(double time, double dt) override;

    /**
     * @brief Builds the tower (aero and elasto part).
     *
     * Sets the nodes and elements for elasto and aero components of the tower, as well as the aero->elasto mapping and
     * elasto->aero mapping.
     */
    void build();

    /**
     * @brief Sets the discretization fractions to use when building the elasto part of the tower.
     *
     * @param[in] fractions Normalized discretization fractions within [0, 1].
     */
    void set_discretization_elasto(std::vector<double> fractions);

    /**
     * @brief Sets the discretization fractions to use when building the aero part of the tower.
     *
     * @param[in] fractions Normalized discretization fractions within [0, 1].
     */
    void set_discretization_aero(std::vector<double> fractions);

    /**
     * @brief Updates aero positions, rotations, velocities and accelerations from elasto component of the tower.
     */
    void update_positions_aero();

    /**
     * @brief Accumulates aero loads to the elasto component of the tower.
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
