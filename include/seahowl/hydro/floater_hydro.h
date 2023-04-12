#pragma once

#include <string>

#include <hydroc/hydro_forces.h>
#include <seahowl/elasto/entities_elasto.h>

namespace seahowl {
namespace hydro {

class FloaterHydroChrono {
  public:
    /** @brief Hydro inputs (from HydroChrono). */
    HydroInputs hydro_inputs;
    /** @brief Path to potential flow frequency data file (HDF5 format). */
    std::string h5_filepath = "";

    /**
     * @brief Constructor.
     */
    FloaterHydroChrono();

    /**
     * @brief Adds body to floater.
     *
     * @param[in] name The name of the body in the hydro file.
     */
    void add_body(std::string& name);

    /**
     * @brief Gets body.
     *
     * @param[in] name The name of the body to return.
     */
    seahowl::elasto::BodyElasto& get_body(std::string& name);

    /**
     * @brief Initialize floater, called before starting the simulation.
     *
     * @param[in] time Time of the simulation (usually 0 at init).
     * @param[in] dt Time step length.
     */
    void initialize(double time, double dt);

  private:
    /** @brief List of bodies and their names. */
    std::map<std::string, std::unique_ptr<seahowl::elasto::BodyElasto>> bodies_map;
    /** @brief HydroChrono logic class. */
    TestHydro hydrochrono_setter;
};

}  // namespace hydro
}  // namespace seahowl
