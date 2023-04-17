#pragma once

#include "seahowl/commons/numerics.h"
#include "seahowl/elasto/entities_elasto.h"
#include "seahowl/elasto/system_elasto.h"

namespace seahowl {
namespace hydro {

class FloaterHydro {
  public:
    /**
     * @brief Assembles the component (adds all bodies to the system).
     *
     * @param[out] system System to which bodies.
     */
    virtual void assemble(seahowl::elasto::SystemElasto& system) = 0;

    /**
     * @brief Initialize floater, called before starting the simulation.
     *
     * @param[in] time Time of the simulation (usually 0 at init).
     * @param[in] dt Time step length.
     */
    virtual void initialize(double time, double dt) = 0;

    /**
     * @brief Returns body to connect to tower.
     */
    virtual seahowl::elasto::BodyElasto& get_tower_connection_body() = 0;

    /**
     * @brief Translates the floater.
     *
     * @param[in] translation_vector The 3D translation vector.
     */
    virtual void translate(Vector3d translation_vector) = 0;

    /**
     * @brief Rotates the floater.
     *
     * @param[in] translation_vector The angle of rotation (in radians).
     * @param[in] axis The axis of rotation (3D vector).
     */
    virtual void rotate(double angle, Vector3d axis) = 0;
};

}  // namespace hydro
}  // namespace seahowl
