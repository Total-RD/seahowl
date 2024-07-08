#include "seahowl/env/combined_models.h"

using namespace seahowl::env;
using seahowl::Vector3d;

double WaveWindModel::get_fluid_density(const Vector3d& position, double time) const {
    if (wave_model->is_in_water(position, time)) {
        return wave_model->get_fluid_density(position, time);
    } else {
        return wind_model->get_fluid_density(position, time);
    }
}

Vector3d WaveWindModel::get_fluid_velocity(const Vector3d& position, double time) const {
    if (wave_model->is_in_water(position, time)) {
        return wave_model->get_fluid_velocity(position, time);
    } else {
        return wind_model->get_fluid_velocity(position, time);
    }
}

Vector3d WaveWindModel::get_fluid_acceleration(const Vector3d& position, double time) const {
    if (wave_model->is_in_water(position, time)) {
        return wave_model->get_fluid_acceleration(position, time);
    } else {
        return wind_model->get_fluid_acceleration(position, time);
    }
}

double FluidSoilModel::get_fluid_density(const Vector3d& position, double time) const {
    return fluid_model->get_fluid_density(position, time);
}

Vector3d FluidSoilModel::get_fluid_velocity(const Vector3d& position, double time) const {
    return fluid_model->get_fluid_velocity(position, time);
}

Vector3d FluidSoilModel::get_fluid_acceleration(const Vector3d& position, double time) const {
    return fluid_model->get_fluid_acceleration(position, time);
}

bool FluidSoilModel::is_in_soil(const Vector3d& position) const {
    return soil_model->is_in_soil(position);
};

Vector3d FluidSoilModel::get_penetration_load(const EntityDynamic& entity,
                                              double contact_area,
                                              double entity_mass) const {
    return soil_model->get_penetration_load(entity, contact_area, entity_mass);
}
