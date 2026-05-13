// SPDX-License-Identifier: Apache-2.0
// Copyright 2022–2026 TotalEnergies
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//

#include "seahowl/env/soil_list_model.h"

using namespace seahowl;
using namespace seahowl::env;

Vector3d SoilListModel::get_penetration_load(const EntityDynamic& entity,
                                             double contact_area,
                                             double entity_mass) const {
    if (models.size() > 0) {
        for (const auto& model : models) {
            if (model->is_inside(entity.get_position())) {
                return model->get_penetration_load(entity, contact_area, entity_mass);
            }
        }
    }
    return Vector3d(0.0, 0.0, 0.0);
}
