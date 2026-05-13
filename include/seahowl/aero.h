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

#include "seahowl/fluid/aero/airfoil.h"
#include "seahowl/fluid/aero/bemt.h"
#include "seahowl/fluid/aero/blade_aero.h"
#include "seahowl/fluid/aero/rotor_aero.h"
#include "seahowl/fluid/aero/tower_aero.h"
#include "seahowl/fluid/aero/reference_point_aero.h"
#ifdef HAVE_AERODYN
    #include "seahowl/fluid/aero/aerodyn_adapter.h"
#endif
