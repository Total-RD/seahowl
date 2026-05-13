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

// Standard library
#include <filesystem>

using path = std::filesystem::path;

// Function to get the test directory from environment variable
path get_test_dir();

// Function to get the data directory from environment variable
path get_data_dir();
