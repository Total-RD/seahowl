#pragma once

#include "seahowl/commons/numerics.h"
#include "seahowl/commons/entities.h"
#include "seahowl/aero/reference_point_aero.h"
#include "seahowl/hydro/morison.h"
#include "seahowl/commons/component_fluid.h"

#include <vector>

// forward declarations
namespace seahowl {
namespace env {
class FluidModel;
}  // namespace env
}  // namespace seahowl

namespace seahowl {

/**@brief Aerodynamic module */
namespace aero {

/**
 * @brief Tower of wind turbine as an aerodynamic component.
 */
class TowerAero : public ComponentFluid {
  public:
    /** @brief Discretization fractions (normalized abscissa) in the range [0, 1] to discretize the aero component. */
    std::vector<double> discretization_fractions;
    /** @brief List of reference points describing the tower properties along its longitudinal axis. */
    std::vector<TowerReferencePointAero> reference_points;
    /** @brief List of discretized points (interpolated reference points) describing the tower properties. */
    std::vector<TowerReferencePointAero> discretized_points;
    /** @brief Aero nodes. */
    std::vector<hydro::MorisonNode> nodes;
    /** @brief Aero elements. */
    std::vector<hydro::MorisonElement> elements;
    /** @brief Loads at center of tower elements. */
    std::vector<Vector3d> loads;

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
    void compute_fluid_loads(const env::FluidModel& fluid_model, double time);
};

}  // namespace aero
}  // namespace seahowl
