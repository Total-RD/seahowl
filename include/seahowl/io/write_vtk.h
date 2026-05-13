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
#include "seahowl/elasto/component_elasto.h"

// Forward declaration
class vtkUnstructuredGrid;
namespace seahowl {
namespace core {
class System;
}  // namespace core
}  // namespace seahowl

namespace seahowl {
namespace io {
/** @brief Output mesh with VTK format */
struct OutputMeshVTK {
    vtkUnstructuredGrid* mesh;

    seahowl::elasto::ComponentElastoFEA& component;

    double time = 0.0;
    double dt = 0.0;
    std::string base = "";

    OutputMeshVTK(seahowl::elasto::ComponentElastoFEA& component);
    OutputMeshVTK(const OutputMeshVTK&);
    OutputMeshVTK(OutputMeshVTK&&) noexcept;
    OutputMeshVTK& operator=(const OutputMeshVTK&) = delete;
    OutputMeshVTK& operator=(OutputMeshVTK&&) = delete;

    ~OutputMeshVTK();

    void initialize(const char* base_name);
    void write(double time, int time_step) const;
};

/** @brief Output system with VTK format */
class OutputSystemVTK {
  public:
    OutputSystemVTK(seahowl::core::System& system_core, const std::string& output_folder);
    void initialize();
    void write(int step);

  private:
    seahowl::core::System& system_core;
    std::string output_folder;
    std::vector<OutputMeshVTK> vtk_meshes;
};
}  // namespace io
}  // namespace seahowl
