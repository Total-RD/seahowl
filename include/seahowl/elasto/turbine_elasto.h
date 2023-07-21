#pragma once

#include "seahowl/elasto/rotor_elasto.h"
#include "seahowl/elasto/tower_elasto.h"

#include <vector>

namespace seahowl {
namespace elasto {
class SystemElasto;
}  // namespace elasto
}  // namespace seahowl

/**@brief Seahowl base namespace */
namespace seahowl {

/**@brief Seahowl elasto module */
namespace elasto {

/**
 * @brief Wind turbine (blades, rotor-nacelle assembly, tower).
 *
 * This class controls each component, ensuring proper workflow for the elasto part.
 */
class TurbineElasto {
  public:
    // components
    //
    /** @brief Rotor-nacelle assembly of the turbine. */
    seahowl::elasto::RotorNacelleAssemblyElasto rna;
    /** @brief Tower of the turbine. */
    seahowl::elasto::TowerElasto tower;

    /**
     * @brief Constructor.
     *
     * Instantiates rotor component and tower component.
     */
    TurbineElasto();

    /**
     * @brief Assembles the turbine (elasto part).*
     *
     * Calls assemble for each of the components of the turbine.
     *
     * @param[out] system System on which to add bodies, links, etc.
     */
    void assemble(seahowl::elasto::SystemElasto& system);

    /**
     * @brief Links RNA to tower.
     *
     * This function links the towertop node to the yaw bearing rigid body by translating the RNA so that the tower
     * towertop node and yaw bearing coordinates match each other.
     * The link between towertop node and yaw bearing is fixed.
     */
    void link_rna_tower(seahowl::elasto::SystemElasto& system);

    /**
     * @brief Builds the turbine.
     *
     * Calls build for each of the components of the turbine.
     */
    void build();

    /**
     * @brief Translates the turbine.
     *
     * @param[in] translation_vector The 3D translation vector.
     */
    void translate(seahowl::Vector3d translation_vector);

    /**
     * @brief Rotates the turbine.
     *
     * @param[in] translation_vector The angle of rotation (in radians).
     * @param[in] axis The axis of rotation (3D vector).
     */
    void rotate(double angle, seahowl::Vector3d axis);
};

}  // namespace elasto
}  // namespace seahowl
