#pragma once

#include "seahowl/commons/numerics.h"
#include "seahowl/elasto/entities_elasto.h"
#include "seahowl/elasto/system_elasto.h"

#include <deque>

namespace seahowl {
namespace elasto {

class FloaterElasto : public ComponentElasto {
  public:
    std::deque<std::unique_ptr<seahowl::elasto::BodyElasto>> fairleads;
    std::deque<std::unique_ptr<seahowl::elasto::Link>> links_fairlead_floater;

    /**
     * @brief Adds fairlead to system.
     *
     * @param[out] position Position of fairlead.
     */
    virtual void add_fairlead(Vector3d& position) = 0;

    /**
     * @brief Returns body to connect to tower.
     */
    virtual seahowl::elasto::BodyElasto& get_tower_connection_body() = 0;
};

class FloaterElastoRigid : public FloaterElasto {
  public:
    std::unique_ptr<seahowl::elasto::BodyElasto> floater_body;

    /**
     * @brief Constructor.
     */
    FloaterElastoRigid();

    /**
     * @brief Assembles the component (adds all bodies to the system).
     *
     * @param[out] system System to which bodies.
     */
    virtual void assemble(seahowl::elasto::SystemElasto& system) override;

    /**
     * @brief Adds fairlead to system.
     *
     * @param[out] position Position of fairlead.
     */
    virtual void add_fairlead(Vector3d& position) override;

    /**
     * @brief Returns body to connect to tower.
     */
    virtual seahowl::elasto::BodyElasto& get_tower_connection_body() override;

    /**
     * @brief Translates the floater.
     *
     * @param[in] translation_vector The 3D translation vector.
     */
    virtual void translate(const Vector3d& translation_vector) const override;

    /**
     * @brief Rotates the floater.
     *
     * @param[in] translation_vector The angle of rotation (in radians).
     * @param[in] axis The axis of rotation (3D vector).
     */
    virtual void rotate(double angle, const Vector3d& axis) const override;

    /**
     * @brief Returns the mass of the floater.
     */
    virtual double get_mass() const override;
};

}  // namespace elasto
}  // namespace seahowl
