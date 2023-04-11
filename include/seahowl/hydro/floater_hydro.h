#pragma once

#include <string>

#include <hydroc/hydro_forces.h>
#include <seahowl/elasto/entities_elasto.h>

class FloaterHydroChrono {
  public:
    /** @brief Hydro inputs (from HydroChrono). */
    HydroInputs hydro_inputs;
    /** @brief Path to potential flow frequency data file (HDF5 format). */
    std::string filepath_h5_potential = "";

    /**
     * @brief Sets file path for potential flow frequency data in HDF5 format.
     *
     * @param[in] filepath Filepath to HDF5 file.
     */
    void set_h5filepath(const std::string& filepath);

    /**
     * @brief Adds body to floater.
     *
     * @param[in] body The body to pass (will be owned by floater).
     * @param[in] body The name of the body in the hydro file.
     */
    void add_body(std::unique_ptr<seahowl::elasto::BodyElasto>&& body, std::string& name);

    /**
     * @brief Gets body.
     *
     * @param[in] body The name of the body to return.
     */
    seahowl::elasto::BodyElasto& get_body(std::string& name);

    /**
     * @brief Initializes floater.
     */
    void initialize();

  private:
    /** @brief List of bodies and their names. */
    std::map<std::string, std::unique_ptr<seahowl::elasto::BodyElasto>> bodies_map;
    /** @brief HydroChrono logic class. */
    TestHydro hydrochrono_setter;
};
