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
    /** @brief Center of mass (COM/COG) position relative to rotor apex (in rotor reference frame). */
    Vector3d position_from_apex{0.0, 0.0, 0.0};
    /** @brief Mass of the hub. */
    double mass = 0.0;
    /** @brief Inertia of the hub. */
    Eigen::Matrix<double, 3, 3> inertia = Eigen::Matrix<double, 3, 3>::Zero();
    /** @brief Overhang of the hub (horizontal distance from towertop). */
    double overhang = 0.0;
    /** @brief Radius of hub (from hub apex to hub edge in rotor plane). */
    double radius = 0.0;
};

/**
 * @brief Nacelle properties.
 */
struct NacelleProperties {
    /** @brief Center of mass (COM/COG) position from towertop (in towertop reference frame). */
    Vector3d position_from_towertop{0.0, 0.0, 0.0};
    /** @brief Mass of the nacelle. */
    double mass = 0.0;
    /** @brief Inertia of the nacelle. */
    Eigen::Matrix<double, 3, 3> inertia = Eigen::Matrix<double, 3, 3>::Zero();
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
    /** @brief Actuator for forced hub rotation (disabled by default). */
    std::unique_ptr<ActuatorRotation> actuator_hub;
    /** @brief Link between hub and actuator. */
    std::unique_ptr<Link> link_hub;

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
     * @brief Builds the rotor.
     */
    void build() override;
    virtual void presetup(double fraction) override;
    virtual void rotate(double angle, const Vector3d& axis) const override;
    virtual void translate(const Vector3d& translation_vector) const override;
    virtual double get_mass() const override;

    /**
     * @brief Resets accumulated loads.
     */
    void reset_loads();

    /**
     * @brief Applies pitch increment to all blades (i.e. rotates blades around their respective longitudinal axis).
     *
     * @param[in] pitch_increment Pitch increment to apply (in radians).
     */
    void apply_collective_pitch_increment(double pitch_increment);

    /**
     * @brief Applies pitch increment on a given blade (i.e. rotates the blade around its longitudinal axis).
     *
     * @param[in] pitch_increment Pitch increment to apply (in radians).
     * @param[in] blade_index Index of blade to pitch (0, 1, or 2 for a 3-bladed turbine).
     */
    void apply_blade_pitch_increment(double pitch_increment, int blade_index);

    /**
     * @brief Accumulates torque on the rotor.
     *
     * @param[in] torque Torque to accumulate on axial axis of hub.
     */
    void accumulate_axial_torque(double torque);

  protected:
    virtual void assemble_this(seahowl::elasto::SystemElasto& system) override;
};

/**
 * @brief Rotor Nacelle Assembly (RNA) of wind turbine as an elasto component.
 *
 * The RNA is composed of rigid bodies (hub, shaft, nacelle, yaw bearing), links (hub-shaft, shaft-nacelle, shaft-yaw
 * bearing, yaw bearing-towertop), and blades (FEA components).
 */
class RotorNacelleAssemblyElasto : public ComponentElasto {
  public:
    /** @brief Initial yaw of the RNA (in radians). */
    double yaw0 = 0.0;

    // RNA components
    //
    /** @brief Rotor. */
    std::unique_ptr<seahowl::elasto::RotorElasto> rotor;
    /** @brief Shaft rigid body. */
    std::unique_ptr<seahowl::elasto::BodyElasto> body_shaft;
    /** @brief Nacelle rigid body. */
    std::unique_ptr<seahowl::elasto::BodyElasto> body_nacelle;
    /** @brief Actuator for yaw dynamics. */
    std::unique_ptr<ActuatorRotation> actuator_yaw;

    // links
    //
    /** @brief Link between shaft and hub (revolute). */
    std::unique_ptr<Link> link_shaft_hub;
    /** @brief Link between shaft and nacelle (fixed). */
    std::unique_ptr<Link> link_shaft_nacelle;
    /** @brief Link between shaft and yaw bearing (fixed). */
    std::unique_ptr<Link> link_shaft_yaw_bearing;
    /** @brief Link between RNA and mounting point (fixed). */
    std::unique_ptr<Link> link_rna;
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
     * @brief Builds the rotor.
     */
    void build() override;

    virtual void presetup(double fraction) override;
    void rotate(double angle, const Vector3d& axis) const override;     ///< @see ElastoComponent::rotate
    void translate(const Vector3d& translation_vector) const override;  ///< @see ElastoComponent::translate
    double get_mass() const override;                                   ///< @see ElastoComponent::get_mass

    /**
     * @brief Resets accumulated loads.
     */
    void reset_loads();

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
    void accumulate_electrical_torque(double torque);

    /**
     * @brief Returns the electrical torque applied on the rotor.
     */
    double get_electrical_torque() const;

    /**
     * @brief Applies yaw increment to the RNA (i.e. rotates the RNA around its mounting points).
     *
     * @param yaw_increment Yaw increment value (in radians).
     */
    void apply_yaw_increment(double yaw_increment);

    /**
     * @brief Returns current of the RNA.
     */
    double get_yaw() const;

    /**
     * @brief Attaches RNA to body on mounting point.
     *
     * @param body Body on which the RNA is attached.
     */
    void attach_rna_to_body(const BodyElasto& body);

    /**
     * @brief Attaches RNA to node on mounting point.
     *
     * @param node Node on which the RNA is attached.
     */
    void attach_rna_to_node(const NodeElasto& node);

  protected:
    virtual void assemble_this(seahowl::elasto::SystemElasto& system) override;

  private:
    /** @brief Accumulated Electrical torque on the rotor.*/
    double torque_elec_accumulated;
    /** @brief Whether the RNA is mounted (e.g. on a tower) or not. */
    bool is_mounted = false;
};

}  // namespace elasto
}  // namespace seahowl
