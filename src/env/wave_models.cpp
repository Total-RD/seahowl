#include "seahowl/env/wave_models.h"

using namespace seahowl::env;
using seahowl::Vector3d;

Vector3d StillWater::get_fluid_velocity(const Vector3d& position, double time) const {
    if (is_in_water(position, time)) {
        return Vector3d(0.0, 0.0, 0.0);
    } else {
        throw std::runtime_error("Cannot retrieve water velocity above mean water level.");
    }
}

double StillWater::get_fluid_density(const Vector3d& position, double time) const {
    if (is_in_water(position, time)) {
        return density;
    } else {
        throw std::runtime_error("Cannot retrieve water density above mean water level.");
    }
}

bool StillWater::is_in_water(const Vector3d& position, double time) const {
    if (position[2] <= mean_water_level) {
        return true;
    } else {
        return false;
    }
}
