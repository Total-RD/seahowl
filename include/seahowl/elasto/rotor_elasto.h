#pragma once

#include "seahowl/elasto/component_elasto.h"
#include "seahowl/commons/numerics.h"

#include <vector>
#include <memory>

// forward declarations
namespace seahowl {
namespace elasto {
class BodyElasto;
class BladeElasto;
class SystemElasto;
}  // namespace elasto
}  // namespace seahowl

namespace seahowl {
namespace elasto {

/**
 * @brief Hub properties.
 */
struct HubProperties {
    /** @brief Center of mass (COM/COG) offset. */
    double center_of_mass = 0.0;
    /** @brief Mass of the hub. */
    double mass = 0.0;
    /** @brief Inertia of the hub. */
    double inertia = 0.0;
    /** @brief Overhang of the hub (horizontal distance from towertop). */
    double overhang = 0.0;
    /** @brief Radius of hub (from hub apex to hub edge in rotor plane). */
    double radius = 0.0;
};

/**
 * @brief Nacelle properties.
 */
struct NacelleProperties {
    /** @brief Center of mass (COM/COG). */
    Vector3d center_of_mass{0.0, 0.0, 0.0};
    /** @brief Mass of the nacelle. */
    double mass = 0.0;
    /** @brief Inertia of the nacelle (@todo include 3x3 inertia). */
    double inertia = 0.0;
    /** @brief Yaw bearing mass. */
    double yaw_bearing_mass = 0.0;
};

/**
 *@brief Shaft properties.
 */
struct ShaftProperties {
    /** @brief Tilt angle of the shaft (radians). */
    double tilt = 0.0;
    /** @brief Distance of shaft axis from towertop. */
    double distance_from_towertop = 0.0;
};

class RotorElasto : public ComponentElasto {
  public:
    /** @brief List of blades. */
    std::vector<std::shared_ptr<seahowl::elasto::BladeElasto>> blades{};
    /** @brief Hub rigid body. */
    std::unique_ptr<seahowl::elasto::BodyElasto> body_hub;
    /** @brief Links between blades and hub. */
    std::vector<std::unique_ptr<Link>> links_blades{};

    /** @brief Hub reference properties. */
    HubProperties hub;
    /** @brief Blades precones (in radians). */
    std::vector<double> blade_precones;

    /** @brief Collective pitch of blades (in radians). */
    double pitch_collective = 0;

    /**
     * @brief Constructor.
     */
    RotorElasto();

    /**
     * @brief Assembles the component (adds all rigid bodies and links to the system).
     *
     * @param[out] system System to which rigid bodies and links are added.
     */
    void assemble(seahowl::elasto::SystemElasto& system);

    /**
     * @brief Builds the rotor.
     */
    void build();

    virtual void rotate(double angle, const Vector3d& axis) const override;
    virtual void translate(const Vector3d& translation_vector) const override;
    virtual double get_mass() const override;

    /**
     * @brief Applies pitch increment to all blades (i.e. rotates blades around their respective longitudinal axis).
     *
     * This function links the towertop node to the yaw bearing rigid body by translating the RNA so that the tower
     * towertop node and yaw bearing coordinates match each other.
     * The link between towertop node and yaw bearing is fixed.
     *
     * @param[in] pitch_increment Pitch increment to apply (in radians).
     */
    void apply_collective_pitch_increment(double pitch_increment);
};

/**
 * @brief Rotor Nacelle Assembly (RNA) of wind turbine as an elasto component.
 *
 * The RNA is composed of rigid bodies (hub, shaft, nacelle, yaw bearing), links (hub-shaft, shaft-nacelle, shaft-yaw
 * bearing, yaw bearing-towertop), and blades (FEA components).
 */
class RotorNacelleAssemblyElasto : public ComponentElasto {
  public:
    // RNA components
    //
    /** @brief Rotor. */
    std::unique_ptr<seahowl::elasto::RotorElasto> rotor;
    /** @brief Shaft rigid body. */
    std::unique_ptr<seahowl::elasto::BodyElasto> body_shaft;
    /** @brief Nacelle rigid body. */
    std::unique_ptr<seahowl::elasto::BodyElasto> body_nacelle;
    /** @brief Yaw bearing rigid body. */
    std::unique_ptr<seahowl::elasto::BodyElasto> body_yaw_bearing;

    // links
    //
    /** @brief Link between shaft and hub (revolute). */
    std::unique_ptr<Link> link_shaft_hub;
    /** @brief Link between shaft and nacelle (fixed). */
    std::unique_ptr<Link> link_shaft_nacelle;
    /** @brief Link between shaft and yaw bearing (fixed). */
    std::unique_ptr<Link> link_shaft_yaw_bearing;
    /** @brief Link between towertop (if any) and yaw bearing (fixed). */
    std::unique_ptr<Link> link_towertop_yaw_bearing;
    ///@}

    // reference properties
    //
    /** @brief Shaft reference properties. */
    ShaftProperties shaft;
    /** @brief Nacelle reference properties. */
    NacelleProperties nacelle;

    /**
     * @brief Constructor.
     */
    RotorNacelleAssemblyElasto();

    /**
     * @brief Assembles the component (adds all rigid bodies and links to the system).
     *
     * @param[out] system System to which rigid bodies and links are added.
     */
    void assemble(seahowl::elasto::SystemElasto& system);

    /**
     * @brief Builds the rotor.
     */
    void build();

    void rotate(double angle, const Vector3d& axis) const override;     ///< @see ElastoComponent::rotate
    void translate(const Vector3d& translation_vector) const override;  ///< @see ElastoComponent::translate
    double get_mass() const override;                                   ///< @see ElastoComponent::get_mass

    /**
     * @brief Returns the RPM of the rotor.
     */
    double get_rpm() const;

    /**
     * @brief Returns the axial torque of the rotor.
     */
    double get_axial_torque() const;

    /**
     * @brief Returns the axial thrust of the rotor.
     */
    double get_axial_thrust() const;

    /**
     * @brief Returns azimuth of rotor.
     */
    double get_azimuth() const;

    /**
     * @brief Accumulates torque on the rotor.
     *
     * @param[in] torque Torque to accumulate on axial axis of hub.
     */
    void accumulate_axial_torque(double torque);
};

}  // namespace elasto
}  // namespace seahowl
