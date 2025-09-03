#pragma once

#include "seahowl/commons/numerics.h"
#include "seahowl/elasto/component_elasto.h"
#include "seahowl/elasto/entities_elasto.h"

namespace seahowl {
namespace elasto {

class FoundationElasto : public virtual ComponentElasto {
  public:
    /**
     * @brief Links foundation to entity.
     *
     * @param[in] entity Entity to which the foundation will be linked.
     */
    virtual void link_to_entity(const Entity& entity) = 0;

    /**
     * @brief Fixes foundation in space.
     *
     * param[in] is_fixed Fixed if true, free if false.
     */
    virtual void set_fixed(bool is_fixed) = 0;

    /**
     * @brief Returns whether foundation is fixed (true) or not (false).
     */
    virtual bool is_fixed() const = 0;

  protected:
    /** @brief Whether foundation is linked to entity or not. */
    bool is_linked = false;
};

class FoundationElastoBody : public virtual FoundationElasto {
  public:
    /** @brief Body of foundation. */
    std::unique_ptr<BodyElasto> body_foundation;
    /** @brief Link between foundation and entity (e.g. towerbase of turbine). */
    std::unique_ptr<Link> link_foundation_entity;

    FoundationElastoBody();

    virtual void link_to_entity(const Entity& entity) override;
    virtual void set_fixed(bool is_fixed) override;
    virtual bool is_fixed() const override;
    virtual void build() override;
    virtual void translate(const Vector3d& translation_vector) const override;
    virtual void rotate(double angle, const Vector3d& axis) const override;
    virtual double get_mass() const override;

  protected:
    virtual void assemble_this(seahowl::elasto::SystemElasto& system) override;
};

}  // namespace elasto
}  // namespace seahowl
