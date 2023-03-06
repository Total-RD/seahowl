#pragma once

#include <seahowl/elasto/strategy_elasto.h>
#include <seahowl/elasto/chrono_adapters.h>

#include <memory.h>

namespace seahowl {
namespace elasto {

class StrategyElastoChrono : public StrategyElasto {
  public:
    bool fpm_mode = false;
    StrategyElastoChrono(){};
    virtual std::shared_ptr<NodeElasto> make_node(Vector3d position, Quaternion rotation) const override {
        return std::make_shared<NodeElastoChrono>(position, rotation);
    };
    virtual std::shared_ptr<ElementElasto> make_element_blade() const override {
        if (fpm_mode == false) {
            return std::make_shared<ElementBladeElastoChrono>();
        } else {
            return std::make_shared<ElementBladeElastoChronoFPM>();
        }
    };
    virtual std::shared_ptr<ElementElasto> make_element_tower() const override {
        return std::make_shared<ElementBladeElastoChrono>();
    };
    virtual std::shared_ptr<ElementElasto> make_element_mooring() const override {
        return std::make_shared<ElementMooringElastoChrono>();
    };
    virtual std::shared_ptr<BodyElasto> make_body() const override { return std::make_shared<BodyElastoChrono>(); };
    virtual std::shared_ptr<Link> make_link() const override { return std::make_shared<LinkChrono>(); };
};

}  // namespace elasto
}  // namespace seahowl
