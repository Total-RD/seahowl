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

#include "seahowl/fluid/hydro/morison.h"
#include "seahowl/fluid/hydro/foundation_fluid.h"
#include "seahowl/fluid/hydro/floater_hydro.h"
#include "seahowl/fluid/hydro/monopile_hydro.h"
#include "seahowl/fluid/hydro/mooring_hydro.h"
#ifdef HAVE_HYDROCHRONO
    #include "seahowl/fluid/hydro/hydrochrono_adapter.h"
#endif
#ifdef HAVE_HYDRODYN
    #include "seahowl/fluid/hydro/hydrodyn_adapter.h"
#endif
