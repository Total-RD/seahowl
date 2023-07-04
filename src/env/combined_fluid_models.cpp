#include "seahowl/env/combined_fluid_models.h"

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
