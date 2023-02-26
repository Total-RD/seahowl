#pragma once

#include <seahowl/aero/reference_point_aero.h>
#include <seahowl/aero/wind_models.h>
#include <seahowl/core/utils.h>

#include <chrono/core/ChVector.h>

namespace seahowl {

/**@brief Aerodynamic module */
namespace aero {

/**
 * @brief Blade aerodynamic node.
 */
struct BladeNodeAero {
    /** @brief Coordinates of node. */
    chrono::ChVector<double> coordinates;
    /** @brief Rotation of node. */
    chrono::ChQuaternion<double> rotation;
    /** @brief Translational velocity of node. */
    chrono::ChVector<double> velocity;
    /** @brief Rotational velocity (global) of node. */
    chrono::ChVector<double> rot_velocity;
    /** @brief Translational acceleration of node. */
    chrono::ChVector<double> acceleration;
    /** @brief Rotational accelation (global) of node. */
    chrono::ChVector<double> rot_acceleration;
    /** @brief Load calculated at node. */
    chrono::ChVector<double> load;
    /** @brief Uninduced wind velocity at node. */
    chrono::ChVector<double> wind_velocity;
    /** @brief Tower-shadowed wind velocity at node. */
    chrono::ChVector<double> wind_velocity_shadowed;
    /** @brief Induced wind velocity at node. */
    chrono::ChVector<double> relative_velocity_induced;
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
    chrono::ChVector<double> get_offset_aero_absolute() const;

    /**
     * @brief Returns induced velocity.
     *
     * @param[in] local_velocity_rotor0 Local uninduced velocity at node.
     * @param[in] blade_pitch Pitch of blade on which node is placed.
     * @param[in] nblades Number of blade.
     * @param[in] tip_loss Whether to take tip loss into account or not.
     * @param[in] hub_loss Whether to take hub loss into account or not.
     */
    chrono::ChVector2<double> get_induced_velocity_rotor(const chrono::ChVector2<double>& local_velocity_rotor0,
                                                         double blade_pitch = 0.0,
                                                         size_t nblades = 3,
                                                         bool tip_loss = true,
                                                         bool hub_loss = true);
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
    double fraction;
    /** @brief Length of element. */
    double length;
    /** @brief Offset (x, y) for the aerodynamic center of blade at center of element. */
    chrono::ChVector2<double> offset_aero;

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
    chrono::ChVector<double> get_load() const;

    /**
     * @brief Get position of center of element.
     */
    chrono::ChVector<double> get_position() const;

    /**
     * @brief Get rotation of center of element.
     */
    chrono::ChQuaternion<double> get_rotation() const;

    /**
     * @brief Get aero offset of center of element in global frame of reference.
     */
    chrono::ChVector<double> get_offset_aero_absolute() const;
};

/**
 * @brief Blade of wind turbine as an aerodynamic component.
 *
 * Blades are discretized into elements each between 2 nodes. Aero loads are calculated at nodes and then linearly
 * integrated over element (and considered as a single point force in the center of the element).
 */
class BladeAero {
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
    std::vector<chrono::ChVector<double>> loads;
    /** @brief Initial azimuth of the blade relative to rotor azimuth (in radians). */
    double azimuth0 = 0.0;
    /** @brief Pitch of the blade (in radians). */
    double pitch = 0.0;

    /**
     * @brief Constructor.
     */
    BladeAero();

    /**
     * @brief Builds the blade.
     */
    void build();

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
    void compute_distances_from_hub(const chrono::ChVector<double>& hub_apex_position, double hub_radius);

    /**
     * @brief Computes node radii.
     *
     * @param[in] hub_apex_position Position of the hub's apex.
     */
    void compute_radii(const chrono::ChVector<double>& hub_apex_position);

    /**
     * @brief Returns average wind velocity along blade.
     */
    chrono::ChVector<double> get_average_wind_velocity();

    /**
     * @brief Returns total aero load on blade.
     */
    chrono::ChVector<double> get_total_load();
};

}  // namespace aero
}  // namespace seahowl
