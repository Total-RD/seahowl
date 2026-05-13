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
#include "seahowl/io/viz_insitu.h"

namespace chrono {
class ChSystem;
namespace irrlicht {
class ChVisualSystemIrrlicht;
}  // namespace irrlicht
}  // namespace chrono

namespace seahowl {
namespace io {
class VisualizationInSituIrrlicht : public VisualizationInSitu {
  public:
    VisualizationInSituIrrlicht();

    void initialize(seahowl::core::System& system) override;
    void initialize_elasto(seahowl::elasto::SystemElasto& system_elasto) override;
    void draw() override;

  private:
    std::shared_ptr<chrono::irrlicht::ChVisualSystemIrrlicht> application_irrlicht;
    std::shared_ptr<chrono::ChSystem> system_chrono;
};
}  // namespace io
}  // namespace seahowl
