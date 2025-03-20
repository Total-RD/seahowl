#include "seahowl/env/fluid_list_model.h"

using namespace seahowl;
using namespace seahowl::env;

double FluidListModel::get_density(const Vector3d& position, double time) const {
    if (models.size() > 0) {
        for (const auto& model : models) {
            if (model->is_inside(position, time)) {
                return model->get_density(position, time);
            }
        }
    }
    return 0.0;
}

Vector3d FluidListModel::get_velocity(const Vector3d& position, double time) const {
    if (models.size() > 0) {
        for (const auto& model : models) {
            if (model->is_inside(position, time)) {
                return model->get_velocity_inside(position, time);
            }
        }
    }
    return Vector3d(0.0, 0.0, 0.0);
}

Vector3d FluidListModel::get_acceleration(const Vector3d& position, double time) const {
    if (models.size() > 0) {
        for (const auto& model : models) {
            if (model->is_inside(position, time)) {
                return model->get_acceleration_inside(position, time);
            }
        }
    }
    return Vector3d(0.0, 0.0, 0.0);
}
