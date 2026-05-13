// SPDX-License-Identifier: Apache-2.0
// Copyright 2022–2026 TotalEnergies
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//

#include "seahowl/env/soil_models.h"

using namespace seahowl::env;
using seahowl::Vector3d;

LinearSoilModel::LinearSoilModel() {}

bool LinearSoilModel::is_inside(const Vector3d& position, double time) const {
    double penetration_depth = (soil_position - position.dot(soil_normal));
    if (penetration_depth >= 0) {
        return true;
    }
    return false;
}

Vector3d LinearSoilModel::get_penetration_load(const EntityDynamic& entity,
                                               double contact_area,
                                               double entity_mass) const {
    auto load = Vector3d(0.0, 0.0, 0.0);
    auto position = entity.get_position();
    auto velocity = entity.get_velocity();
    double penetration_depth = (soil_position - position.dot(soil_normal));
    if (penetration_depth >= 0) {
        // stiffness force
        load += stiffness_normal * penetration_depth * contact_area * soil_normal;

        // damping force
        auto lambda = 0.5;  // fraction of critical damping
        double penetration_velocity = (velocity).dot(-soil_normal);
        if (penetration_depth >= 0 && penetration_velocity > 0) {
            load += 2.0 * lambda * std::sqrt(stiffness_normal * entity_mass * contact_area) * penetration_velocity *
                    soil_normal;
        }

        Vector3d tangential_velocity = velocity - penetration_velocity * (-soil_normal);
        if (penetration_depth >= 0 && tangential_velocity.norm() > 0) {
            load += -2.0 * lambda * std::sqrt(stiffness_shear * entity_mass * contact_area) * tangential_velocity;
        }
    }
    return load;
}
