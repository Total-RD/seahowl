#pragma once

#include "seahowl/io/viz_insitu.h"
#include "seahowl/io/write_csv.h"
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

/** Class for managing simulation outputs (CSV, VTK, in situ, logs etc).
 */
class OutputManager {
  public:
    /** @brief Time step for generating outputs. */
    double dt_output = 0.0;
    /** @brief Whether manager outputs VTK files or not. */
    bool has_vtk = false;
    /** @brief Whether manager displays in situ visualization or not. */
    bool has_gui = false;
    /** @brief Whether manager outputs CSV files or not. */
    bool has_csv = true;

    /**
     * @brief Constructor.
     *
     * @param system_core System from which the output manager grabs information.
     */
    OutputManager(seahowl::core::System& system_core);

    /**
     * @brief Sets main folder for outputs.
     *
     * @param output_folder Path to output folder.
     */
    void set_output_folder(const std::string& output_folder);

    /**
     * @brief Pre-initializes outputs, needs to be called before system initialization.
     *
     * This is used to set some output-related that need to be set before initializing system.
     * For example, the flag for AeroDyn's VTK output needs to be set before initializing TurbineAeoDyn.
     */
    void preinitialize();

    /**
     * @brief Initializes outputs.
     */
    void initialize();

    /**
     * @brief Outputs everything at current time step.
     *
     * @param step Current step iteration (used for output names such as VTK).
     */
    void output_all(int step);

    /**
     * @brief Outputs initial logs.
     */
    void output_initial_logs();

    /**
     * @brief Adds and returns a reference to a new CustomCSV in the list of CSVs.
     *
     * @param csv_filename Name (or path) of custom CSV, relative to output_folder.
     */
    seahowl::io::CustomCSV& create_new_csv(const std::string& csv_filepath);

  private:
    bool is_initialized = false;
    std::string output_folder = "./output";
    seahowl::core::System& system_core;
    std::vector<CustomCSV> custom_csv_list;
#ifdef HAVE_VTK
    std::unique_ptr<OutputSystemVTK> output_vtk;
#endif
    std::unique_ptr<VisualizationInSitu> output_insitu;
};
}  // namespace io
}  // namespace seahowl
