#pragma once

#include "seahowl/elasto/rotor_elasto.h"
#include "seahowl/elasto/tower_elasto.h"

#include <vector>

// forward declarations
namespace seahowl {
namespace elasto {
class SystemElasto;
class FoundationElasto;
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
class TurbineElasto : public ComponentElasto {
  public:
    // components
    //
    /** @brief Rotor-nacelle assembly of the turbine. */
    seahowl::elasto::RotorNacelleAssemblyElasto rna;
    /** @brief Tower of the turbine. */
    seahowl::elasto::TowerElasto tower;
    /** @brief Foundation of the turbine */
    std::shared_ptr<seahowl::elasto::FoundationElasto> foundation;

    /**
     * @brief Constructor.
     *
     * Instantiates rotor component and tower component.
     */
    TurbineElasto();

    /**
     * @brief Links RNA to tower.
     *
     * This function links the towertop node to the yaw bearing rigid body by translating the RNA so that the tower
     * towertop node and yaw bearing coordinates match each other.
     * The link between towertop node and yaw bearing is fixed.
     */
    void link_rna_tower();

    /**
     * @brief Presetup of turbine.
     *
     * @param[in] fraction Fraction of presetup phase, starting at 0.0 and ending at 1.0.
     */
    virtual void presetup(double fraction) override;

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
    virtual void translate(const seahowl::Vector3d& translation_vector) const override;

    /**
     * @brief Rotates the turbine.
     *
     * @param[in] translation_vector The angle of rotation (in radians).
     * @param[in] axis The axis of rotation (3D vector).
     */
    virtual void rotate(double angle, const seahowl::Vector3d& axis) const override;

    /**
     * @brief Returns the mass of the turbine.
     */
    virtual double get_mass() const override;

  protected:
    virtual void assemble_this(seahowl::elasto::SystemElasto& system) override;
};

}  // namespace elasto
}  // namespace seahowl
