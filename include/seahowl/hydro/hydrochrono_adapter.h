#pragma once

#include "seahowl/elasto/floater_elasto.h"
#include "seahowl/elasto/entities_elasto.h"
#include "seahowl/elasto/chrono_adapters.h"
#include "seahowl/elasto/floater_elasto.h"
#include "seahowl/env/wave_models.h"

#include <string>

#include <hydroc/hydro_forces.h>

namespace seahowl {
namespace hydro {

/**
 * @brief HydroChrono adapter for floater class.
 */
class FloaterHydroChrono : public elasto::FloaterElasto {
  public:
    /**
     * @brief Constructor.
     */
    FloaterHydroChrono();

    /**
     * @brief Initialize floater, called before starting the simulation.
     *
     * @param[in] time Time of the simulation (usually 0 at init).
     * @param[in] dt Time step length.
     */
    virtual void initialize() override;

    /**
     * @brief Sets path to .h5 file containing hydro data.
     *
     * @param[in] filepath Path to .h5 file.
     */
    void set_h5_filepath(std::string filepath);

    /**
     * @brief Sets waves used to compute hydro loads on floater.
     *
     * @param[in] waves Waves to attach to floater.
     */
    void set_waves(std::shared_ptr<WaveBase> waves);

  private:
    /** @brief Waves (HydroChrono). */
    std::shared_ptr<WaveBase> waves;
    /** @brief HydroChrono logic class. */
    std::unique_ptr<TestHydro> hydrochrono_setter;
    /** @brief Path to potential flow frequency data file (HDF5 format). */
    std::string h5_filepath = "";
};

}  // namespace hydro

namespace env {

/**
 * @brief Class for HydroChrono waves.
 */
class WaveModelHydroChrono : public WaveModel {
  public:
    /** @brief Waves (HydroChrono). */
    std::shared_ptr<WaveBase> waves;

    /**
     * @brief Constructor.
     */
    WaveModelHydroChrono();

    virtual Vector3d get_fluid_velocity(const Vector3d& position, double time) const override;
    virtual double get_fluid_density(const Vector3d& position, double time) const override;
    virtual bool is_in_water(const Vector3d& position, double time) const override;
};

}  // namespace env

}  // namespace seahowl
