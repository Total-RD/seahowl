#pragma once

#include <seahowl/elasto/component_elasto.h>

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
    double height;
    /** @brief Height of the base of the tower (absolute value above ground / sea water level). */
    double base_height;

    /**
     * @brief Constructor.
     */
    TowerElasto();

    /**
     * @brief Builds the blade (to call before assemble).
     */
    void build();

  private:
    /**
     * @brief Builds the blade with Timoshenko elements (lineic density, foreaft stiffness, sideside stiffness).
     */
    void build_elements();
};

}  // namespace elasto
}  // namespace seahowl
