#pragma once

#include "seahowl/io/viz_insitu.h"
#ifdef HAVE_VTK
    #include "seahowl/io/write_vtk.h"
#endif

#include <memory>
#include <string>

// Forward declaration
namespace seahowl {
namespace core {
class System;
}  // namespace core
}  // namespace seahowl

namespace seahowl {
namespace io {
class OutputManager {
  public:
    bool has_vtk = false;
    bool has_gui = false;
    bool has_csv = true;

    OutputManager(seahowl::core::System& system_core);
    void set_output_folder(const std::string& output_folder);
    void initialize();
    void output_all(int step);

  private:
    bool is_initialized = false;
    std::string output_folder = ".";
    seahowl::core::System& system_core;
#ifdef HAVE_VTK
    std::unique_ptr<OutputSystemVTK> output_vtk;
#endif
    std::unique_ptr<VisualizationInSitu> output_insitu;
};
}  // namespace io
}  // namespace seahowl
