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

// Third-party libraries
#include <Eigen/Dense>

/** @brief SEAHOWL library namespace. */
namespace seahowl {

/** @brief Pi constant. */
constexpr double PI = 3.14159265358979323846;

/** @brief Vector 3D. */
using Vector3d = Eigen::Vector3d;

/** @brief Vector 2D. */
using Vector2d = Eigen::Vector2d;

/** @brief Quaternion. */
using Quaternion = Eigen::Quaterniond;

/** @brief Angle-axis rotation representation. */
using AngleAxisd = Eigen::AngleAxisd;

}  // namespace seahowl
