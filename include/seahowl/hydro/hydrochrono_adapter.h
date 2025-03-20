#pragma once

#include "seahowl/elasto/floater_elasto.h"
#include "seahowl/elasto/entities_elasto.h"
#include "seahowl/elasto/chrono_adapters.h"
#include "seahowl/elasto/floater_elasto.h"
#include "seahowl/env/wave_models.h"

#include <string>

// forward declarations for HydroChrono
class WaveBase;
class TestHydro;
// namespace hydroc

namespace seahowl {
namespace env {
class WaveModelHydroChrono;
}  // namespace env
}  // namespace seahowl

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
    void set_waves_hydrochrono(std::shared_ptr<WaveBase> waves);

    /**
     * @brief Sets waves used to compute hydro loads on floater.
     *
     * @param[in] waves Waves to attach to floater.
     */
    void set_waves(std::shared_ptr<seahowl::env::WaveModelHydroChrono> waves);

  private:
    /** @brief Waves (HydroChrono). */
    std::shared_ptr<WaveBase> waves;
    /** @brief HydroChrono logic class. */
    std::shared_ptr<TestHydro> hydrochrono_setter;
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

    virtual double get_water_level(const Vector3d& position, double time) const override;

  protected:
    virtual double get_density_this(const Vector3d& position, double time) const override;
    virtual Vector3d get_velocity_this(const Vector3d& position, double time) const override;
    virtual Vector3d get_acceleration_this(const Vector3d& position, double time) const override;
};

}  // namespace env

}  // namespace seahowl
