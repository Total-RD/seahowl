// SPDX-License-Identifier: Apache-2.0
// Copyright 2022–2026 TotalEnergies
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//

#include "seahowl/env/wave_models.h"

using namespace seahowl::env;
using seahowl::Vector3d;

bool WaveModel::is_inside(const Vector3d& position, double time) const {
    if (position.dot(surface_normal) <= get_water_level(position, time)) {
        return true;
    } else {
        return false;
    }
}

StillWater::StillWater() {}

Vector3d StillWater::get_velocity_this(const Vector3d& position, double time) const {
    return Vector3d(0.0, 0.0, 0.0);
}

Vector3d StillWater::get_acceleration_this(const Vector3d& position, double time) const {
    return Vector3d(0.0, 0.0, 0.0);
}

double StillWater::get_density_this(const Vector3d& position, double time) const {
    return density;
}

double StillWater::get_water_level(const Vector3d& position, double time) const {
    return mean_water_level;
}

CurrentConstant::CurrentConstant() {}

Vector3d CurrentConstant::get_velocity_this(const Vector3d& position, double time) const {
    auto position_depth = position.dot(surface_normal) - mean_water_level;
    auto horizontal_velocity =
        velocity_seabed +
        (velocity_surface - velocity_seabed) * powf((position_depth + water_depth) / water_depth, 1.0 / power_factor);
    return direction * horizontal_velocity;
}

Vector3d CurrentConstant::get_acceleration_this(const Vector3d& position, double time) const {
    return Vector3d(0.0, 0.0, 0.0);
}
