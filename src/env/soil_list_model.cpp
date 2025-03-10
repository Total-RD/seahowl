#include "seahowl/env/soil_list_model.h"

using namespace seahowl;
using namespace seahowl::env;

Vector3d SoilListModel::get_penetration_load(const EntityDynamic& entity,
                                             double contact_area,
                                             double entity_mass) const {
    if (models.size() > 0) {
        for (const auto& model : models) {
            if (model->is_inside(entity.get_position())) {
                return model->get_penetration_load(entity, contact_area, entity_mass);
            }
        }
    }
    return Vector3d(0.0, 0.0, 0.0);
}
