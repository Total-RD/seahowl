// SPDX-License-Identifier: Apache-2.0
// Copyright 2022–2026 TotalEnergies
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//

#pragma once

#include "seahowl/env/model.h"
#include "seahowl/env/list_model.h"
#include "seahowl/env/env_model.h"
#include "seahowl/env/fluid_list_model.h"
#include "seahowl/env/fluid_models.h"
#include "seahowl/env/soil_list_model.h"
#include "seahowl/env/soil_models.h"
#include "seahowl/env/wave_models.h"
#include "seahowl/env/wind_models.h"
#ifdef HAVE_INFLOWWIND
    #include "seahowl/env/inflowwind_adapter.h"
#endif
#ifdef HAVE_SEASTATE
    #include "seahowl/env/seastate_adapter.h"
#endif
