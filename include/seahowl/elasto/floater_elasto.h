#pragma once

#include "seahowl/commons/numerics.h"
#include "seahowl/elasto/entities_elasto.h"
#include "seahowl/elasto/foundation_elasto.h"
#include "seahowl/elasto/mooring_elasto.h"

#include <deque>
#include <map>

// forward declarations
namespace seahowl {
namespace elasto {
class SystemElasto;
}  // namespace elasto
}  // namespace seahowl

namespace seahowl {
namespace elasto {

class FloaterElasto : public FoundationElasto {
  public:
    /** @brief Link between foundation and entity (e.g. towerbase of turbine). */
    std::unique_ptr<seahowl::elasto::Link> link_floater_entity;
    /** @brief Mooring system of the floater. */
    std::unique_ptr<seahowl::elasto::MooringSystemElasto> mooring_system;
    /** @brief Main body of floater.*/
    std::unique_ptr<seahowl::elasto::BodyElasto> body_main;

    /**
     * @brief Constructor.
     */
    FloaterElasto();

    virtual void link_to_entity(const Entity& entity) override;
    virtual void set_fixed(bool is_fixed) override;
    virtual bool is_fixed() const override;
    virtual void presetup(double fraction) override;
    virtual void build() override;
    virtual void translate(const Vector3d& translation_vector) const override;
    virtual void rotate(double angle, const Vector3d& axis) const override;
    virtual double get_mass() const override;

    /**
     * @brief Creates and adds body to floater.
     *
     * @param[in] name The name of the body (for access purposes).
     */
    void add_body(const std::string& name);

    /**
     * @brief Gets body.
     *
     * @param[in] name The name of the body to return.
     */
    seahowl::elasto::BodyElasto& get_body(const std::string& name);

    /**
     * @brief Adds fairlead to system.
     *
     * @param[in] position Absolute position of fairlead.
     * @param[in] connected_body_name Name of the body connected to the fairlead.
     */
    virtual void add_fairlead(const Vector3d& position, const std::string& connected_body_name);

    /**
     * @brief Returns fairlead count for given body.
     *
     * @param[in] body_name Name of body on which fairleads are connected.
     */
    virtual int get_fairlead_count(const std::string& body_name) const;

    /**
     * @brief Returns fairlead body.
     *
     * @param[in] body_name Name of body on which fairlead is connected.
     * @param[in] index Index of fairlead.
     */
    virtual seahowl::elasto::BodyElasto& get_fairlead_body(const std::string& body_name, int index);

    /**
     * @brief Returns fairlead link.
     *
     * @param[in] body_name Name of body on which fairlead is connected.
     * @param[in] index Index of fairlead.
     */
    virtual seahowl::elasto::Link& get_fairlead_link(const std::string& body_name, int index);

  protected:
    /** @brief List of bodies and their names. */
    std::map<std::string, std::unique_ptr<seahowl::elasto::BodyElasto>> floater_bodies;
    /** @brief List of bodies and their names. */
    std::map<std::string, std::deque<std::unique_ptr<seahowl::elasto::Link>>> fairlead_links;
    /** @brief List of bodies and their names. */
    std::map<std::string, std::deque<std::unique_ptr<seahowl::elasto::BodyElasto>>> fairlead_bodies;
    /** @brief Name of body for tower connection */
    std::string tower_connection_name = "";

    virtual void assemble_this(seahowl::elasto::SystemElasto& system) override;
};

}  // namespace elasto
}  // namespace seahowl
