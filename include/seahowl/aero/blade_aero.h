#pragma once

#include "seahowl/commons/numerics.h"
#include "seahowl/commons/entities.h"
#include "seahowl/aero/reference_point_aero.h"
#include "seahowl/commons/component_fluid.h"

#include <memory>

namespace seahowl {

/**@brief Aerodynamic module */
namespace aero {

/**
 * @brief Blade aerodynamic node.
 */
struct BladeNodeAero : public EntityDynamicEigen {
    /** @brief Load calculated at node. */
    Vector3d load{0.0, 0.0, 0.0};
    /** @brief Moment calculated at node. */
    Vector3d moment{0.0, 0.0, 0.0};
    /** @brief Uninduced wind velocity at node. */
    Vector3d wind_velocity{0.0, 0.0, 0.0};
    /** @brief Tower-shadowed wind velocity at node. */
    Vector3d wind_velocity_shadowed{0.0, 0.0, 0.0};
    /** @brief Induced wind velocity at node. */
    Vector3d relative_velocity_induced{0.0, 0.0, 0.0};
    /** @brief Reference point associated to node (aerodynamic properties). */
    BladeReferencePointAero properties;

    /** @brief Chord solidity (sigma). */
    double chord_solidity = 1.0;
    /** @brief Axial induction factor (first guess for next iteration). */
    double induction_factor_axial = 0.0;
    /** @brief Tangential induction factor (first guess for next iteration). */
    double induction_factor_tangential = 0.0;
    /** @brief Radius (distance from hub apex). */
    double radius = 0.0;
    /** @brief Distance of node from hub (~infinitely far by default for no effect) */
    double distance_from_hub = 99999.9;
    /** @brief Distance of node from blade tip (~infinitely far by default for no effect) */
    double distance_from_tip = 99999.9;

    /**
     * @brief Constructor.
     */
    BladeNodeAero(BladeReferencePointAero& point);

    /**
     * @brief Get aero offset in global frame of reference.
     */
    Vector3d get_offset_aero_absolute() const;
};

/**
 * @brief Blade aerodynamic element.
 */
struct BladeElementAero {
    /** @brief Fist node of element. */
    const BladeNodeAero& node1;
    /** @brief Second node of element. */
    const BladeNodeAero& node2;
    /** @brief Fraction (normalized abscissa along longitudinal axis of component) of center of element. */
    double fraction = 0.0;
    /** @brief Length of element. */
    double length = 0.0;
    /** @brief Offset (x, y) for the aerodynamic center of blade at center of element. */
    Vector2d offset_aero = {0.0, 0.0};

    /**
     * @brief Constructor.
     *
     * @param[in] node1 First node of element.
     * @param[in] node2 Second node of element.
     */
    BladeElementAero(const BladeNodeAero& node1, const BladeNodeAero& node2);

    /**
     * @brief Returns integrated load at center of element.
     */
    Vector3d get_load() const;

    /**
     * @brief Returns integrated moment at center of element.
     */
    Vector3d get_moment() const;

    /**
     * @brief Get position of center of element.
     */
    Vector3d get_position() const;

    /**
     * @brief Get rotation of center of element.
     */
    Quaternion get_rotation() const;

    /**
     * @brief Get aero offset of center of element in global frame of reference.
     */
    Vector3d get_offset_aero_absolute() const;
};

/**
 * @brief Blade of wind turbine as an aerodynamic component.
 *
 * Blades are discretized into elements each between 2 nodes. Aero loads are calculated at nodes and then linearly
 * integrated over element (and considered as a single point force in the center of the element).
 */
class BladeAero : public ComponentFluid {
  public:
    /** @brief Discretization fractions (normalized abscissa) in the range [0, 1] to discretize the aero component. */
    std::vector<double> discretization_fractions;
    /** @brief List of reference points describing the blade properties along its longitudinal axis. */
    std::vector<BladeReferencePointAero> reference_points;
    /** @brief List of discretized points (interpolated reference points) describing the blade properties. */
    std::vector<BladeReferencePointAero> discretized_points;
    /** @brief Aero nodes. */
    std::vector<BladeNodeAero> nodes;
    /** @brief Aero elements. */
    std::vector<BladeElementAero> elements;
    /** @brief Loads at center of blade elements. */
    std::vector<Vector3d> loads;
    /** @brief Loads at center of blade elements. */
    std::vector<Vector3d> moments;
    /** @brief Initial azimuth of the blade relative to rotor azimuth (in radians). */
    double azimuth0 = 0.0;
    /** @brief Pitch of the blade (in radians). */
    double pitch = 0.0;
    /** @brief Body at the root of the blade. */
    std::unique_ptr<EntityDynamic> body_root;

    /**
     * @brief Constructor.
     */
    BladeAero();

    void build() override;
    void compute_fluid_loads(const env::EnvModel& fluid_model, double time) override;

    /**
     * @brief Computes node distances from blade tip.
     */
    void compute_distances_from_tip();

    /**
     * @brief Computes node distances from hub.
     *
     * @param[in] hub_apex_position Position of the hub's apex.
     * @param[in] hub_radius Radius of hub.
     */
    void compute_distances_from_hub(const Vector3d& hub_apex_position, double hub_radius);

    /**
     * @brief Computes node radii.
     *
     * @param[in] hub_apex_position Position of the hub's apex.
     */
    void compute_radii(const Vector3d& hub_apex_position);

    /**
     * @brief Returns average wind velocity along blade.
     */
    Vector3d get_average_wind_velocity();

    /**
     * @brief Returns total aero load on blade.
     */
    Vector3d get_total_load();
};

}  // namespace aero
}  // namespace seahowl
