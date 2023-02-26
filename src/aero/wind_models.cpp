#include "seahowl/aero/wind_models.h"

#pragma once

#include <chrono/core/ChVector.h>

using namespace seahowl::aero;

chrono::ChVector<double> get_sheared_wind_velocity(const chrono::ChVector<double>& velocity,
                                                   const chrono::ChVector<double>& position,
                                                   const chrono::ChVector<double>& direction_gravity,
                                                   double shear_coefficient,
                                                   double reference_height,
                                                   double reference_length) {
    double distance = position ^ (-direction_gravity);
    if (distance > reference_length) {
        distance = reference_length;
    }
    return velocity * pow(distance / reference_height, shear_coefficient);
}

WindModel::WindModel() {}

chrono::ChVector<double> WindModel::get_wind_velocity(const chrono::ChVector<double>& position, double time) const {
    return chrono::ChVector<double>(0.0, 0.0, 0.0);
};

double WindModel::get_density() const {
    return density;
}

ConstantWind::ConstantWind() {
    wind_velocity = chrono::ChVector<double>(0.0, 0.0, 0.0);
    direction_gravity = chrono::ChVector<double>(0.0, 0.0, -1.0);
}

void ConstantWind::set_wind_velocity(chrono::ChVector<double> velocity) {
    wind_velocity = velocity;
}

chrono::ChVector<double> ConstantWind::get_wind_velocity(const chrono::ChVector<double>& position, double time) const {
    auto velocity = get_sheared_wind_velocity(wind_velocity, position, direction_gravity, shear_coefficient,
                                              reference_height, reference_length);
    return velocity;
}

WindRamp::WindRamp() {
    wind_velocity_start = chrono::ChVector<double>(0.0, 0.0, 0.0);
    wind_velocity_stop = chrono::ChVector<double>(0.0, 0.0, 0.0);
    direction_gravity = chrono::ChVector<double>(0.0, 0.0, -1.0);
}

void WindRamp::set_wind_velocity_start(chrono::ChVector<double> velocity) {
    wind_velocity_start = velocity;
}
void WindRamp::set_wind_velocity_stop(chrono::ChVector<double> velocity) {
    wind_velocity_stop = velocity;
}

chrono::ChVector<double> WindRamp::get_wind_velocity(const chrono::ChVector<double>& position, double time) const {
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
