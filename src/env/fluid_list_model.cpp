// SPDX-License-Identifier: Apache-2.0
// Copyright 2022–2026 TotalEnergies
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//

#include "seahowl/env/fluid_list_model.h"

using namespace seahowl;
using namespace seahowl::env;

double FluidListModel::get_density(const Vector3d& position, double time) const {
    for (const auto& model : models) {
        if (model->is_inside(position, time)) {
            return model->get_density(position, time);
        }
    }
    return 0.0;
}

Vector3d FluidListModel::get_velocity(const Vector3d& position, double time) const {
    for (const auto& model : models) {
        if (model->is_inside(position, time)) {
            return model->get_velocity_inside(position, time);
        }
    }
    return Vector3d(0.0, 0.0, 0.0);
}

Vector3d FluidListModel::get_acceleration(const Vector3d& position, double time) const {
    for (const auto& model : models) {
        if (model->is_inside(position, time)) {
            return model->get_acceleration_inside(position, time);
        }
    }
    return Vector3d(0.0, 0.0, 0.0);
}

void FluidListModel::set_ramp(double start_time, double end_time) {
    for (auto& model : models) {
        model->set_ramp(start_time, end_time);
    }
}
