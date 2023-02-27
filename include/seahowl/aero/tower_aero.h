#pragma once

#include <vector>

#include <seahowl/aero/reference_point_aero.h>
#include <seahowl/aero/wind_models.h>
#include <seahowl/core/utils.h>
#include <seahowl/utils.h>

namespace seahowl {

/**@brief Aerodynamic module */
namespace aero {

/**
 * @brief Tower aerodynamic element.
 */
struct TowerElementAero {
    /** @brief Reference point associated to element (aerodynamic properties). */
    TowerReferencePointAero properties;
    /** @brief Length of element. */
    double length = 0.0;

    TowerElementAero(const TowerReferencePointAero& point1, const TowerReferencePointAero& point2);
};

/**
 * @brief Tower of wind turbine as an aerodynamic component.
 */
class TowerAero {
  public:
    /** @brief Discretization fractions (normalized abscissa) in the range [0, 1] to discretize the aero component. */
    std::vector<double> discretization_fractions;
    /** @brief List of reference points describing the tower properties along its longitudinal axis. */
    std::vector<TowerReferencePointAero> reference_points;
    /** @brief List of discretized points (interpolated reference points) describing the tower properties. */
    std::vector<TowerReferencePointAero> discretized_points;
    /** @brief Aero elements. */
    std::vector<TowerElementAero> elements;
    /** @brief Loads at center of tower elements. */
    std::vector<Vector3d> loads;

    /**
     * @brief Constructor.
     */
    TowerAero();

    /**
     * @brief Builds the tower.
     */
    void build();

    /**
     * @brief Compute wind loads on tower using Morison's approach on cylindrical elements.
     */
    void compute_wind_loads_morison(WindModel& wind_model, double time);
};

}  // namespace aero
}  // namespace seahowl
