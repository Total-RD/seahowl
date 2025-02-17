#include "seahowl/env/env_model.h"
#include "seahowl/env/fluid_models.h"
#include "seahowl/env/soil_models.h"
#include "seahowl/env/wave_models.h"

using namespace seahowl::env;
using seahowl::Vector3d;

void EnvModel::addModel(std::shared_ptr<Model> model) {
    models.push_back(model);
}

const std::vector<std::shared_ptr<Model>>& EnvModel::getModels() const {
    return models;
}

double EnvModel::get_fluid_density(const Vector3d& position, double time) const {
    std::vector<std::shared_ptr<FluidModel>> fluid_models = get_models_of_type<FluidModel>();
    if (fluid_models.size() > 0) {
        for (const auto model : fluid_models) {
            if (model->is_inside(position, time)) {
                return model->get_fluid_density(position, time);
            }
        }
    }
    throw std::runtime_error("No fluid model found");
}

Vector3d EnvModel::get_fluid_velocity(const Vector3d& position, double time) const {
    std::vector<std::shared_ptr<FluidModel>> fluid_models = get_models_of_type<FluidModel>();
    if (fluid_models.size() > 0) {
        for (const auto model : fluid_models) {
            if (model->is_inside(position, time)) {
                return model->get_fluid_velocity(position, time);
            }
        }
    }
    throw std::runtime_error("No fluid model found");
}

Vector3d EnvModel::get_fluid_acceleration(const Vector3d& position, double time) const {
    std::vector<std::shared_ptr<FluidModel>> fluid_models = get_models_of_type<FluidModel>();
    if (fluid_models.size() > 0) {
        for (const auto model : fluid_models) {
            if (model->is_inside(position, time)) {
                return model->get_fluid_acceleration(position, time);
            }
        }
    }
    throw std::runtime_error("No fluid model found");
}

Vector3d EnvModel::get_penetration_load(const EntityDynamic& entity, double contact_area, double entity_mass) const {
    std::vector<std::shared_ptr<SoilModel>> soil_models = get_models_of_type<SoilModel>();
    if (soil_models.size() > 0) {
        for (const auto model : soil_models) {
            if (model->is_inside(entity.get_position())) {
                return model->get_penetration_load(entity, contact_area, entity_mass);
            }
        }
    }
    throw std::runtime_error("No soil model found");
}

double EnvModel::get_water_level(const Vector3d& position, double time) const {
    std::vector<std::shared_ptr<WaveModel>> wave_models = get_models_of_type<WaveModel>();
    if (wave_models.size() > 0) {
        for (const auto model : wave_models) {
            if (model->is_inside(position, time)) {
                return model->get_water_level(position, time);
            }
        }
    }
    throw std::runtime_error("No wave model found");
}
