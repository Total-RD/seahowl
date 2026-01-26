#pragma once

#include "seahowl/commons/numerics.h"
#include "seahowl/commons/entities.h"
#include "seahowl/fluid/aero/reference_point_aero.h"
#include "seahowl/fluid/hydro/morison.h"
#include "seahowl/fluid/component_fluid.h"

#include <vector>

// forward declarations
namespace seahowl {
namespace env {
class FluidModel;
}  // namespace env
}  // namespace seahowl

namespace seahowl {
namespace fluid {

/**@brief Aerodynamic module */
namespace aero {

/**
 * @brief Tower of wind turbine as an aerodynamic component.
 */
class TowerAero : public virtual ComponentFluid {
  public:
    /** @brief List of reference points describing the tower properties along its longitudinal axis. */
    std::vector<TowerReferencePointAero> reference_points;
    /** @brief List of discretized points (interpolated reference points) describing the tower properties. */
    std::vector<TowerReferencePointAero> discretized_points;
    /** @brief Aero nodes. */
    std::vector<hydro::MorisonNode> nodes;
    /** @brief Aero elements. */
    std::vector<hydro::MorisonElement> elements;
    /** @brief MacCamy and Fuchs Correction for large cylinders, Flag. */
    bool use_MacCamyFuchs_correction = false;
    /** @brief Cd Correction for large cylinders, Flag. */
    bool use_Cd_correction = false;

    /**
     * @brief Constructor.
     */
    TowerAero();

    /**
     * @brief Builds the tower.
     */
    void build() override;

    /**
     * @brief Compute wind loads on tower using Morison's approach on cylindrical elements.
     */
    void compute_env_loads(const env::EnvModel& env_model, double time) override;
};

}  // namespace aero
}  // namespace fluid
}  // namespace seahowl
