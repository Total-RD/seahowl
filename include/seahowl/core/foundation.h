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
#include "seahowl/core/component.h"

namespace seahowl {
namespace core {

class Foundation : public virtual ComponentDynamic {
  public:
    Foundation(const std::shared_ptr<seahowl::elasto::ComponentElasto> elasto_,
               const std::shared_ptr<seahowl::fluid::ComponentFluid> fluid_)
        : ComponentDynamic(elasto_, fluid_) {}

    virtual ~Foundation() = default;
};

}  // namespace core
}  // namespace seahowl
