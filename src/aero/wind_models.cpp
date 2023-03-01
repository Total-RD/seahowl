#include "seahowl/aero/wind_models.h"

#pragma once

using namespace seahowl::aero;
using seahowl::Vector3d;

Vector3d get_sheared_wind_velocity(const Vector3d& velocity,
                                   const Vector3d& position,
                                   const Vector3d& direction_gravity,
                                   double shear_coefficient,
                                   double reference_height,
                                   double reference_length) {
    double distance = position.dot(-direction_gravity);
    if (distance > reference_length) {
        distance = reference_length;
    }
    return velocity * pow(distance / reference_height, shear_coefficient);
}

WindModel::WindModel() {}

Vector3d WindModel::get_wind_velocity(const Vector3d& position, double time) const {
    return Vector3d(0.0, 0.0, 0.0);
};

double WindModel::get_density() const {
    return density;
}

ConstantWind::ConstantWind() {
    wind_velocity = Vector3d(0.0, 0.0, 0.0);
    direction_gravity = Vector3d(0.0, 0.0, -1.0);
}

void ConstantWind::set_wind_velocity(Vector3d velocity) {
    wind_velocity = velocity;
}

Vector3d ConstantWind::get_wind_velocity(const Vector3d& position, double time) const {
    auto velocity = get_sheared_wind_velocity(wind_velocity, position, direction_gravity, shear_coefficient,
                                              reference_height, reference_length);
    return velocity;
}

WindRamp::WindRamp() {
    wind_velocity_start = Vector3d(0.0, 0.0, 0.0);
    wind_velocity_stop = Vector3d(0.0, 0.0, 0.0);
    direction_gravity = Vector3d(0.0, 0.0, -1.0);
}

void WindRamp::set_wind_velocity_start(Vector3d velocity) {
    wind_velocity_start = velocity;
}
void WindRamp::set_wind_velocity_stop(Vector3d velocity) {
    wind_velocity_stop = velocity;
}

Vector3d WindRamp::get_wind_velocity(const Vector3d& position, double time) const {
    auto velocity = wind_velocity_start;
    if (time >= time_start) {
        double w1 = 1.0 - std::min((time - time_start) / (time_stop - time_start), 1.0);
        double w2 = 1.0 - w1;
        velocity = w1 * wind_velocity_start + w2 * wind_velocity_stop;
    }
    velocity = get_sheared_wind_velocity(velocity, position, direction_gravity, shear_coefficient, reference_height,
                                         reference_length);
    return velocity;
}
