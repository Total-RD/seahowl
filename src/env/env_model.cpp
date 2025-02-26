#include "seahowl/env/env_model.h"

using namespace seahowl::env;
using seahowl::Vector3d;

void EnvModel::add_model(std::shared_ptr<Model> model) {
    models.push_back(model);
    if (std::shared_ptr<FluidModel> fluidmodel = std::dynamic_pointer_cast<FluidModel>(model))
        fluid_models.push_back(fluidmodel);
    if (std::shared_ptr<SoilModel> soilmodel = std::dynamic_pointer_cast<SoilModel>(model))
        soil_models.push_back(soilmodel);
    if (std::shared_ptr<WaveModel> wavemodel = std::dynamic_pointer_cast<WaveModel>(model))
        wave_models.push_back(wavemodel);
}

const std::vector<std::shared_ptr<Model>>& EnvModel::get_models() const {
    return models;
}

double EnvModel::get_fluid_density(const Vector3d& position, double time) const {
    if (fluid_models.size() > 0) {
        for (const auto& model : fluid_models) {
            if (model->is_inside(position, time)) {
                return model->get_density(position, time);
            }
        }
    }
    return 0.0;
}

Vector3d EnvModel::get_fluid_velocity(const Vector3d& position, double time) const {
    if (fluid_models.size() > 0) {
        for (const auto& model : fluid_models) {
            if (model->is_inside(position, time)) {
                return model->get_velocity(position, time);
            }
        }
    }
    return Vector3d(0.0, 0.0, 0.0);
}

Vector3d EnvModel::get_fluid_acceleration(const Vector3d& position, double time) const {
    if (fluid_models.size() > 0) {
        for (const auto& model : fluid_models) {
            if (model->is_inside(position, time)) {
                return model->get_acceleration(position, time);
            }
        }
    }
    return Vector3d(0.0, 0.0, 0.0);
}

Vector3d EnvModel::get_penetration_load(const EntityDynamic& entity, double contact_area, double entity_mass) const {
    if (soil_models.size() > 0) {
        for (const auto& model : soil_models) {
            if (model->is_inside(entity.get_position())) {
                return model->get_penetration_load(entity, contact_area, entity_mass);
            }
        }
    }
    return Vector3d(0.0, 0.0, 0.0);
}

double EnvModel::get_water_level(const Vector3d& position, double time) const {
    if (wave_models.size() > 0) {
        for (const auto& model : wave_models) {
            if (model->is_inside(position, time)) {
                return model->get_water_level(position, time);
            }
        }
    }
    return 0.0;
}
