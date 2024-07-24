#pragma once

#include <memory>
#include <vector>

#include "seahowl/commons/utils.h"  // for DiscretizationPoint
#include "seahowl/core/component.h"

// forward declarations
namespace seahowl {
namespace core {
struct TowerReferencePoint;
}  // namespace core
namespace elasto {
class TowerElasto;
}  // namespace elasto
namespace aero {
class TowerAero;
}  // namespace aero
}  // namespace seahowl

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
    /** @brief Mapping of aero nodes into elasto domain. */
    std::vector<seahowl::DiscretizationPoint> mapping_aero2elasto_nodes;
    /** @brief Mapping of aero elements (central point of elements) into elasto domain. */
    std::vector<seahowl::DiscretizationPoint> mapping_aero2elasto_elements;
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

    void apply_fluid_model(seahowl::env::FluidModel& fluid_model, double time) override;

    /**
     * @brief Builds the tower (aero and elasto part).
     *
     * Sets the nodes and elements for elasto and aero components of the tower, as well as the aero->elasto mapping and
     * elasto->aero mapping.
     */
    void build() override;

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
     * @brief Initialize tower, called before starting the simulation.
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
