#include "seahowl/env/fluid_models.h"

using namespace seahowl::env;
using seahowl::Vector3d;

Vector3d FluidModel::get_velocity(const Vector3d& position, double time) const {
    Vector3d velocity = get_velocity_this(position, time);
    if (time < ramp_end) {
        if (time > ramp_start) {
            double ramp_fraction = (time - ramp_start) / (ramp_end - ramp_start);
            velocity *= ramp_fraction;
        } else {
            velocity *= 0.0;
        }
    }
    return velocity;
};

Vector3d FluidModel::get_acceleration(const Vector3d& position, double time) const {
    Vector3d acceleration = get_acceleration_this(position, time);
    if (time < ramp_end) {
        if (time > ramp_start) {
            double ramp_fraction = (time - ramp_start) / (ramp_end - ramp_start);
            acceleration *= ramp_fraction;
        } else {
            acceleration *= 0.0;
        }
    }
    return acceleration;
};
