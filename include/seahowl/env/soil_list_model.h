#pragma once

#include <vector>
#include <memory>
#include "seahowl/env/soil_models.h"
#include "seahowl/env/list_model.h"

namespace seahowl {
namespace env {
/**
 * @brief Class to store list of soils
 */
class SoilListModel : public ListModel<SoilModel> {
  public:
    /**
     * @brief Returns soil penetration load.
     *
     * @param[in] entity Dynamic entity (potentially) penetrating soil.
     * @param[in] contact_area Contact area of entity penetrating soil.
     * @param[in] entity_mass Mass of entity penetrating soil.
     */
    Vector3d get_penetration_load(const EntityDynamic& entity, double contact_area, double entity_mass) const;
};

}  // namespace env
}  // namespace seahowl
