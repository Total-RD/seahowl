#pragma once

#include "seahowl/elasto/turbine_elasto.h"
#include "seahowl/elasto/floater_elasto.h"
#include "seahowl/elasto/mooring_elasto.h"

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
class TurbineFloatingElasto : public TurbineElasto {
  public:
    // components
    //
    /** @brief Floater of the turbine. */
    std::shared_ptr<seahowl::elasto::FloaterElasto> floater;
    /** @brief Link between floater and tower of the turbine. */
    std::unique_ptr<seahowl::elasto::Link> link_floater_tower;

    /**
     * @brief Constructor.
     *
     * Instantiates rotor component and tower component.
     */
    TurbineFloatingElasto();

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
    void assemble_this(seahowl::elasto::SystemElasto& system) override;
};

}  // namespace elasto
}  // namespace seahowl
