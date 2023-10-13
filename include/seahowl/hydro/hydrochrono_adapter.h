#pragma once

#include "seahowl/elasto/floater_elasto.h"
#include "seahowl/elasto/entities_elasto.h"
#include "seahowl/elasto/chrono_adapters.h"
#include "seahowl/elasto/floater_elasto.h"

#include <string>

#include <hydroc/hydro_forces.h>

namespace seahowl {
namespace hydro {

class FloaterHydroChronoRigid : public elasto::FloaterElastoRigid {
  public:
    /** @brief Waves (HydroChrono). */
    std::shared_ptr<WaveBase> waves;

    /**
     * @brief Constructor.
     */
    FloaterHydroChronoRigid();

    /**
     * @brief Initialize floater, called before starting the simulation.
     *
     * @param[in] time Time of the simulation (usually 0 at init).
     * @param[in] dt Time step length.
     */
    virtual void initialize(double time, double dt) override;

    /**
     * @brief Sets name of floater.
     *
     * @param[in] name Name of the body in the hydro file.
     */
    void set_name(std::string& name);

    /**
     * @brief Sets path to .h5 file containing hydro data.
     *
     * @param[in] h5_filepath Path to .h5 file.
     */
    void set_h5_filepath(std::string& h5_filepath);

  private:
    /** @brief HydroChrono logic class. */
    std::unique_ptr<TestHydro> hydrochrono_setter;
    /** @brief Path to potential flow frequency data file (HDF5 format). */
    std::string h5_filepath = "";
    /** @brief Name of HydroChrono/BEM body. */
    std::string floater_body_name = "";
};

class FloaterHydroChrono : public elasto::FloaterElasto {
  public:
    /** @brief Waves (HydroChrono). */
    std::shared_ptr<WaveBase> waves;
    /** @brief Path to potential flow frequency data file (HDF5 format). */
    std::string h5_filepath = "";

    /**
     * @brief Constructor.
     */
    FloaterHydroChrono();

    /**
     * @brief Adds fairlead to system.
     *
     * @param[out] position Position of fairlead.
     */
    virtual void add_fairlead(Vector3d& position) override;

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
     * @brief Returns list of body names in floater.
     */
    std::vector<std::string> get_body_names_list();

    /**
     * @brief Assembles the component (adds all bodies to the system).
     *
     * @param[out] system System to which bodies.
     */
    virtual void assemble(seahowl::elasto::SystemElasto& system) override;

    /**
     * @brief Initialize floater, called before starting the simulation.
     *
     * @param[in] time Time of the simulation (usually 0 at init).
     * @param[in] dt Time step length.
     */
    virtual void initialize(double time, double dt) override;

    /**
     * @brief Returns body to connect to tower.
     */
    virtual seahowl::elasto::BodyElasto& get_tower_connection_body() override;

    /**
     * @brief Translates the floater.
     *
     * @param[in] translation_vector The 3D translation vector.
     */
    virtual void translate(Vector3d translation_vector) override;

    /**
     * @brief Rotates the floater.
     *
     * @param[in] translation_vector The angle of rotation (in radians).
     * @param[in] axis The axis of rotation (3D vector).
     */
    virtual void rotate(double angle, Vector3d axis) override;

  private:
    /** @brief List of bodies and their names. */
    std::map<std::string, std::unique_ptr<seahowl::elasto::BodyElasto>> bodies_map;
    /** @brief HydroChrono logic class. */
    std::unique_ptr<TestHydro> hydrochrono_setter;
};

}  // namespace hydro
}  // namespace seahowl
