#include "seahowl/env/fluid_models.h"

using namespace seahowl::env;
using seahowl::Vector3d;

Vector3d FluidModel::get_velocity_inside(const Vector3d& position, double time) const {
    Vector3d velocity = get_velocity_this(position, time);
    apply_ramp(time, velocity);
    return velocity;
};

Vector3d FluidModel::get_velocity(const Vector3d& position, double time) const {
    if (is_inside(position, time)) {
        return get_velocity_inside(position, time);
    } else {
        return Vector3d(0.0, 0.0, 0.0);
    }
};

Vector3d FluidModel::get_acceleration_inside(const Vector3d& position, double time) const {
    Vector3d acceleration = get_acceleration_this(position, time);
    apply_ramp(time, acceleration);
    return acceleration;
};

Vector3d FluidModel::get_acceleration(const Vector3d& position, double time) const {
    if (is_inside(position, time)) {
        return get_acceleration_inside(position, time);
    } else {
        return Vector3d(0.0, 0.0, 0.0);
    }
};

double FluidModel::get_density_inside(const Vector3d& position, double time) const {
    return get_density_this(position, time);
};

double FluidModel::get_density(const Vector3d& position, double time) const {
    if (is_inside(position, time)) {
        return get_density_inside(position, time);
    } else {
        return 0.0;
    }
};

void seahowl::env::FluidModel::apply_ramp(double time, Vector3d& res) const {
    if (time < ramp_end) {
        if (time > ramp_start) {
            double ramp_fraction = (time - ramp_start) / (ramp_end - ramp_start);
            res *= ramp_fraction;
        } else {
            res *= 0.0;
        }
    }
}
