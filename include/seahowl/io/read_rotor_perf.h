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

// SEAHOWL headers
#include "seahowl/core/turbine.h"
#include "seahowl/fluid/aero/rotor_aero.h"

// Standard library
#include <memory>
#include <string>
#include <vector>

/**
 * @brief Returns rotor disk performance tables (Cp, Cq, Ct).
 *
 * @param[in] filepath Path of the .txt file describing the rotor disk performance.
 * @param[in] aero Object of class seahowl::aero::RotorAero
 */
void get_disk_perf_from_table(std::string filepath, seahowl::aero::RotorAeroDisk& aero);
