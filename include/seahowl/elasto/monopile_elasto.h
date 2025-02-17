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
    MonopileElasto() : TowerElasto() {
        body_tp = std::make_unique<BodyElastoChrono>();
        link_tp_entity = std::make_unique<seahowl::elasto::LinkChrono>();
        link_tp_monopile = std::make_unique<seahowl::elasto::LinkChrono>();
    };

    virtual void link_to_entity(const Entity& entity) override {
        link_tp_entity->initialize(*body_tp, entity);
        link_tp_entity->set_constraints(true, true, true, true, true, true);
        link_tp_monopile->initialize(*body_tp, *nodes.back());
        link_tp_monopile->set_constraints(true, true, true, true, true, true);
        is_linked = true;
    };

  protected:
    /** @brief Whether floater is linked to entity or not. */
    bool is_linked = false;

    virtual void assemble_this(seahowl::elasto::SystemElasto& system) override {
        TowerElasto::assemble_this(system);
        if (is_linked) {
            system.add(*link_tp_entity);
            system.add(*link_tp_monopile);
            system.add(*body_tp);
        }
    };
};

}  // namespace elasto
}  // namespace seahowl
