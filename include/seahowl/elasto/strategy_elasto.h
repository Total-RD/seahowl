#pragma once

#include <seahowl/elasto/entities_elasto.h>
#include <memory>

namespace seahowl {
namespace elasto {

class StrategyElasto {
  public:
    virtual std::shared_ptr<NodeElasto> make_node(Vector3d position, Quaternion rotation) const = 0;
    virtual std::shared_ptr<ElementElasto> make_element_blade() const = 0;
    virtual std::shared_ptr<ElementElasto> make_element_tower() const = 0;
    virtual std::shared_ptr<ElementElasto> make_element_mooring() const = 0;
    virtual std::shared_ptr<BodyElasto> make_body() const = 0;
    virtual std::shared_ptr<Link> make_link() const = 0;
};

}  // namespace elasto
}  // namespace seahowl
