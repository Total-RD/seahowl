#include "seahowl/env/wind_models.h"

using namespace seahowl::env;
using seahowl::Vector3d;

Vector3d get_sheared_wind_velocity(const Vector3d& velocity,
                                   const Vector3d& position,
                                   const Vector3d& direction_gravity,
                                   double shear_coefficient,
                                   double reference_height) {
    double distance = position.dot(-direction_gravity);
    // assuming that ground level is at 0.0
    if (distance < 0) {
        distance = 0;
    }
    return velocity * pow(distance / reference_height, shear_coefficient);
}

bool WindModel::is_inside(const Vector3d& position, double time) const {
    return true;
}

double WindModel::get_density_this(const Vector3d& position, double time) const {
    return density;
}

ConstantWind::ConstantWind() {
    wind_velocity = Vector3d(0.0, 0.0, 0.0);
    direction_gravity = Vector3d(0.0, 0.0, -1.0);
}

void ConstantWind::set_wind_velocity(Vector3d velocity) {
    wind_velocity = velocity;
}

Vector3d ConstantWind::get_velocity_this(const Vector3d& position, double time) const {
    auto velocity =
        get_sheared_wind_velocity(wind_velocity, position, direction_gravity, shear_coefficient, reference_height);
    return velocity;
}

Vector3d ConstantWind::get_acceleration_this(const Vector3d& position, double time) const {
    return Vector3d(0.0, 0.0, 0.0);
}

WindRamp::WindRamp() {}

void WindRamp::set_wind_ramp(const Vector3d& velocity_start,
                             double time_start,
                             const Vector3d& velocity_end,
                             double time_end) {
    wind_velocity_start = velocity_start;
    this->time_start = time_start;
    wind_velocity_end = velocity_end;
    this->time_end = time_end;
}

Vector3d WindRamp::get_velocity_this(const Vector3d& position, double time) const {
    auto velocity = wind_velocity_start;
    if (time >= time_start) {
        if (time_end != time_start) {
            double w1 = 1.0 - std::min((time - time_start) / (time_end - time_start), 1.0);
            double w2 = 1.0 - w1;
            velocity = w1 * wind_velocity_start + w2 * wind_velocity_end;
        } else if (time_end == time_start) {
            velocity = wind_velocity_end;
        } else {
            std::runtime_error("End time is less than start time for the wind ramp.");
        }
    }
    velocity = get_sheared_wind_velocity(velocity, position, direction_gravity, shear_coefficient, reference_height);
    return velocity;
}

Vector3d WindRamp::get_acceleration_this(const Vector3d& position, double time) const {
    auto acceleration = Vector3d(0.0, 0.0, 0.0);
    if (time > time_start && time < time_end) {
        acceleration = (wind_velocity_end - wind_velocity_start) / (time_end - time_start);
    }
    return acceleration;
}
