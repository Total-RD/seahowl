#pragma once

#include "seahowl/elasto/foundation_elasto.h"
#include "seahowl/elasto/tower_elasto.h"
#include "seahowl/elasto/chrono_adapters.h"

namespace seahowl {
namespace elasto {

/**
 * @brief Monopile of wind turbine as an elastodynamic FEA component.
 *
 * Monopiles, like towers, are discretized into beam elements that are either simple Timoshenko elements described
 * through lineic density, foreaft and sideside stiffnesses.
 */
class MonopileElasto : public TowerElasto, public virtual FoundationElasto {
  public:
    /** @brief Body of transition piece (TP). */
    std::unique_ptr<BodyElastoChrono> body_tp;
    /** @brief Link between TP and entity (e.g. towerbase of turbine). */
    std::unique_ptr<seahowl::elasto::Link> link_tp_entity;
    /** @brief Link between monopile and TP. */
    std::unique_ptr<seahowl::elasto::Link> link_tp_monopile;

    /**
     * @brief Constructor.
     */
    MonopileElasto();

    virtual void link_to_entity(const Entity& entity) override;

    virtual void build() override;

    /**
     * @brief Translates the monopile.
     *
     * @param[in] translation_vector The 3D translation vector.
     */
    virtual void translate(const Vector3d& translation_vector) const override;

    /**
     * @brief Rotates the monopile.
     *
     * @param[in] translation_vector The angle of rotation (in radians).
     * @param[in] axis The axis of rotation (3D vector).
     */
    virtual void rotate(double angle, const Vector3d& axis) const override;

    /**
     * @brief Returns the mass of the monopile.
     */
    virtual double get_mass() const override;

  protected:
    /** @brief Whether monopile is linked to entity or not. */
    bool is_linked = false;

    virtual void assemble_this(seahowl::elasto::SystemElasto& system);
};

}  // namespace elasto
}  // namespace seahowl
