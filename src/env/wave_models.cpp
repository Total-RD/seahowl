#include "seahowl/env/wave_models.h"

using namespace seahowl::env;
using seahowl::Vector3d;

bool WaveModel::is_in_water(const Vector3d& position, double time) const {
    if (position.dot(surface_normal) <= get_water_level(position, time)) {
        return true;
    } else {
        return false;
    }
}

StillWater::StillWater() {}

Vector3d StillWater::get_fluid_velocity(const Vector3d& position, double time) const {
    if (is_in_water(position, time)) {
        return Vector3d(0.0, 0.0, 0.0);
    } else {
        throw std::runtime_error("Cannot retrieve water velocity above mean water level.");
    }
}

Vector3d StillWater::get_fluid_acceleration(const Vector3d& position, double time) const {
    if (is_in_water(position, time)) {
        return Vector3d(0.0, 0.0, 0.0);
    } else {
        throw std::runtime_error("Cannot retrieve water acceleration above mean water level.");
    }
}

double StillWater::get_fluid_density(const Vector3d& position, double time) const {
    if (is_in_water(position, time)) {
        return density;
    } else {
        throw std::runtime_error("Cannot retrieve water density above mean water level.");
    }
}

double StillWater::get_water_level(const Vector3d& position, double time) const {
    return mean_water_level;
}

CurrentConstant::CurrentConstant() {}

Vector3d CurrentConstant::get_fluid_velocity(const Vector3d& position, double time) const {
    if (is_in_water(position, time)) {
        auto position_depth = position.dot(surface_normal) - mean_water_level;
        auto horizontal_velocity =
            velocity_seabed + (velocity_surface - velocity_seabed) *
                                  powf((position_depth + water_depth) / water_depth, 1.0 / power_factor);
        return direction * horizontal_velocity;
    } else {
        throw std::runtime_error("Cannot retrieve water velocity above mean water level.");
    }
}

Vector3d CurrentConstant::get_fluid_acceleration(const Vector3d& position, double time) const {
    if (is_in_water(position, time)) {
        return Vector3d(0.0, 0.0, 0.0);
    } else {
        throw std::runtime_error("Cannot retrieve water acceleration above mean water level.");
    }
}
