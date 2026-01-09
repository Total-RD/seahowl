#pragma once

#include "seahowl/elasto/component_elasto.h"
#include "seahowl/elasto/reference_point_elasto.h"

namespace seahowl {
namespace elasto {

/**
 * @brief Tower of wind turbine as an elastodynamic FEA component.
 *
 * Towers are discretized into beam elements that are either simple Timoshenko elements described through lineic
 * density, foreaft and sideside stiffnesses.
 */
class TowerElasto : public ComponentElastoFEA {
  public:
    /** @brief List of reference points describing the tower properties along its longitudinal axis. */
    std::vector<TowerReferencePointElasto> reference_points;
    /** @brief List of discretized points (interpolated reference points) describing the tower properties. */
    std::vector<TowerReferencePointElasto> discretized_points;
    /** @brief Height of the tower (absolute value above ground / sea water level). */
    double height = 0.0;
    /** @brief Height of the base of the tower (absolute value above ground / sea water level). */
    double base_height = 0.0;

    /**
     * @brief Constructor.
     */
    TowerElasto();

    /**
     * @brief Builds the blade (to call before assemble).
     */
    void build() override;

    /**
     * @brief Returns tower base moment (first node of first element of tower).
     */
    Vector3d get_tower_base_moment() const;

    /**
     * @brief Returns tower base moment (second node of last element of tower).
     */
    Vector3d get_tower_top_moment() const;

    /**
     * @brief Returns tower base force (first node of first element of tower).
     */
    Vector3d get_tower_base_force() const;

    /**
     * @brief Returns tower base force (second node of last element of tower).
     */
    Vector3d get_tower_top_force() const;

    /**
     * @brief Resets accumulated loads at nodes of tower component.
     */
    virtual void reset_loads() override;

  private:
    /**
     * @brief Builds the blade with Timoshenko elements (lineic density, foreaft stiffness, sideside stiffness).
     */
    void build_elements_tapered_timoshenko();
};

}  // namespace elasto
}  // namespace seahowl
