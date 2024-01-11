#pragma once

#include "seahowl/commons/numerics.h"
#include "seahowl/commons/entities.h"
#include "seahowl/aero/reference_point_aero.h"

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
 * @brief Tower aerodynamic node.
 */
struct TowerNodeAero : public EntityDynamicEigen {
    /** @brief Load calculated at node. */
    Vector3d load{0.0, 0.0, 0.0};
    /** @brief Reference point associated to node (aerodynamic properties). */
    TowerReferencePointAero properties;

    /**
     * @brief Constructor.
     */
    TowerNodeAero(TowerReferencePointAero& point);
};

/**
 * @brief Tower aerodynamic element.
 */
struct TowerElementAero {
    /** @brief Fist node of element. */
    const TowerNodeAero& node1;
    /** @brief Second node of element. */
    const TowerNodeAero& node2;
    /** @brief Fraction (normalized abscissa along longitudinal axis of component) of center of element. */
    double fraction = 0.0;
    /** @brief Length of element. */
    double length = 0.0;

    TowerElementAero(const TowerNodeAero& point1, const TowerNodeAero& point2);

    /**
     * @brief Returns integrated load at center of element.
     */
    Vector3d get_load() const;

    /**
     * @brief Get position of center of element.
     */
    Vector3d get_position() const;

    /**
     * @brief Get rotation of center of element.
     */
    Quaternion get_rotation() const;
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
    /** @brief Aero nodes. */
    std::vector<TowerNodeAero> nodes;
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
    void compute_aero_loads(const env::FluidModel& wind_model, double time);
};

}  // namespace aero
}  // namespace seahowl
