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

double SoilWaveWindModel::get_fluid_density(const Vector3d& position, double time) const {
    if (soil_model->is_in_soil(position)) {
        return 0.0;
    } else if (wave_model->is_in_water(position, time)) {
        return wave_model->get_fluid_density(position, time);
    } else {
        return wind_model->get_fluid_density(position, time);
    }
}

Vector3d SoilWaveWindModel::get_fluid_velocity(const Vector3d& position, double time) const {
    if (soil_model->is_in_soil(position)) {
        return Vector3d(0.0, 0.0, 0.0);
    } else if (wave_model->is_in_water(position, time)) {
        return wave_model->get_fluid_velocity(position, time);
    } else {
        return wind_model->get_fluid_velocity(position, time);
    }
}

Vector3d SoilWaveWindModel::get_penetration_load(const EntityDynamic& entity,
                                                 double contact_area,
                                                 double entity_mass) const {
    return soil_model->get_penetration_load(entity, contact_area, entity_mass);
}
