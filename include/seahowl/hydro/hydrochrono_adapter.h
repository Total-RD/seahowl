#pragma once

#include "seahowl/elasto/floater_elasto.h"
#include "seahowl/elasto/entities_elasto.h"
#include "seahowl/elasto/chrono_adapters.h"
#include "seahowl/elasto/floater_elasto.h"

#include <string>

#include <hydroc/hydro_forces.h>

namespace seahowl {
namespace hydro {

class FloaterHydroChrono : public elasto::FloaterElasto {
  public:
    /** @brief Waves (HydroChrono). */
    std::shared_ptr<WaveBase> waves;

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
    void set_h5_filepath(std::string& filepath);

  private:
    /** @brief HydroChrono logic class. */
    std::unique_ptr<TestHydro> hydrochrono_setter;
    /** @brief Path to potential flow frequency data file (HDF5 format). */
    std::string h5_filepath = "";
};

}  // namespace hydro
}  // namespace seahowl
