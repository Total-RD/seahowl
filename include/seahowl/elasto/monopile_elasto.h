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
