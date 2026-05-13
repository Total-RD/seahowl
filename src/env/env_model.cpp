// SPDX-License-Identifier: Apache-2.0
// Copyright 2022–2026 TotalEnergies
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//

#include "seahowl/env/env_model.h"

// SEAHOWL headers
#include "seahowl/env/model.h"
#include "seahowl/env/wind_models.h"

using namespace seahowl::env;
using seahowl::Vector3d;

void EnvModel::add_model(const std::shared_ptr<Model>& model) {
    if (std::shared_ptr<FluidModel> fluidmodel = std::dynamic_pointer_cast<FluidModel>(model))
        fluid_models.insert_model<WindModel>(fluidmodel);
    if (std::shared_ptr<SoilModel> soilmodel = std::dynamic_pointer_cast<SoilModel>(model))
        soil_models.insert_model<SoilModel>(soilmodel);
}
