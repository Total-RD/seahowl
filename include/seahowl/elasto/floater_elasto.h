#pragma once

#include "seahowl/commons/numerics.h"
#include "seahowl/elasto/entities_elasto.h"
#include "seahowl/elasto/system_elasto.h"

#include <deque>
#include <map>

namespace seahowl {
namespace elasto {

class FloaterElasto : public ComponentElasto {
  public:
    /**
     * @brief Constructor.
     */
    FloaterElasto();

    /**
     * @brief Assembles the component (adds all bodies to the system).
     *
     * @param[out] system System to which bodies.
     */
    virtual void assemble(seahowl::elasto::SystemElasto& system) override;

    /**
     * @brief Creates and adds body to floater.
     *
     * @param[in] name The name of the body (for access purposes).
     */
    seahowl::elasto::BodyElasto& add_body(std::string& name);

    /**
     * @brief Gets body.
     *
     * @param[in] name The name of the body to return.
     */
    seahowl::elasto::BodyElasto& get_body(std::string& name);

    /**
     * @brief Adds fairlead to system.
     *
     * @param[in] position Absolute position of fairlead.
     * @param[in] connected_body_name Name of the body connected to the fairlead.
     */
    virtual void add_fairlead(Vector3d& position, std::string connected_body_name);

    /**
     * @brief Sets name of body that will be used for tower connection.
     *
     * @param[in] connected_body_name Name of the body to connect to the tower.
     */
    virtual void set_tower_connection_body_name(std::string connected_body_name);

    /**
     * @brief Returns body to connect to tower.
     */
    virtual seahowl::elasto::BodyElasto& get_tower_connection_body() const;

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

  protected:
    /** @brief List of bodies and their names. */
    std::map<std::string, std::unique_ptr<seahowl::elasto::BodyElasto>> floater_bodies;
    /** @brief List of bodies and their names. */
    std::map<std::string, std::vector<std::unique_ptr<seahowl::elasto::Link>>> fairlead_links;
    /** @brief List of bodies and their names. */
    std::map<std::string, std::vector<std::unique_ptr<seahowl::elasto::BodyElasto>>> fairlead_bodies;
    /** @brief Name of body for tower connection */
    std::string tower_connection_name = "";
};

}  // namespace elasto
}  // namespace seahowl
