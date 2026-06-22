// SPDX-License-Identifier: Apache-2.0
// Copyright 2022–2026 TotalEnergies
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//

#pragma once

// SEAHOWL headers
#include "seahowl/commons/numerics.h"
#include "seahowl/elasto/component_elasto.h"

// Standard library
#include <memory>
#include <vector>

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
    /** @brief Center of mass (COM/COG) position relative to rotor apex (in rotor reference frame) [m] */
    Vector3d position_from_apex{0.0, 0.0, 0.0};
    /** @brief Mass of the hub [kg] */
    double mass = 0.0;
    /** @brief Inertia of the hub [kg.m^2] */
    Eigen::Matrix<double, 3, 3> inertia = Eigen::Matrix<double, 3, 3>::Zero();
    /** @brief Overhang of the hub (horizontal distance from towertop) [m] */
    double overhang = 0.0;
    /** @brief Radius of hub (from hub apex to hub edge in rotor plane) [m] */
    double radius = 0.0;
};

/**
 * @brief Nacelle properties.
 */
struct NacelleProperties {
    /** @brief Center of mass (COM/COG) position from towertop (in towertop reference frame) [m] */
    Vector3d position_from_towertop{0.0, 0.0, 0.0};
    /** @brief Mass of the nacelle [kg] */
    double mass = 0.0;
    /** @brief Inertia of the nacelle [kg.m^2] */
    Eigen::Matrix<double, 3, 3> inertia = Eigen::Matrix<double, 3, 3>::Zero();
    /** @brief Yaw bearing mass [kg] */
    double yaw_bearing_mass = 0.0;
};

/**
 *@brief Shaft properties.
 */
struct ShaftProperties {
    /** @brief Tilt angle of the shaft [rad] */
    double tilt = 0.0;
    /** @brief Distance of shaft axis from towertop [m] */
    double distance_from_towertop = 0.0;
};

class RotorElasto : public ComponentElasto {
  public:
    /** @brief List of blades. */
    std::vector<std::shared_ptr<seahowl::elasto::BladeElasto>> blades{};
    /** @brief Hub rigid body. */
    std::unique_ptr<seahowl::elasto::BodyElasto> body_hub;

    /** @brief Hub reference properties. */
    HubProperties hub;
    /** @brief Blades precones [rad] */
    std::vector<double> blade_precones;

    /** @brief Collective pitch of blades [rad] */
    double pitch_collective = 0;

    /**
     * @brief Constructor.
     */
    RotorElasto();

    void build() override;
    virtual void presetup(double fraction) override;
    virtual void rotate(double angle, const Vector3d& axis) const override;
    virtual void translate(const Vector3d& translation_vector) const override;
    virtual double get_mass() const override;
    void reset_loads() override;

    /**
     * @brief Applies pitch increment to all blades, rotating them around their respective longitudinal axis.
     *
     * @param[in] pitch_increment Pitch increment to apply [rad]
     */
    void apply_collective_pitch_increment(double pitch_increment);

    /**
     * @brief Accumulates torque on the rotor.
     *
     * @param[in] torque Torque to accumulate on axial axis of hub [Nm]
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
    /** @brief Initial yaw of the RNA [rad] */
    double yaw0 = 0.0;

    // RNA components
    //
    /** @brief Rotor. */
    std::shared_ptr<seahowl::elasto::RotorElasto> rotor;
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
    RotorNacelleAssemblyElasto(std::shared_ptr<seahowl::elasto::RotorElasto> rotor);
    ~RotorNacelleAssemblyElasto() = default;

    void build() override;
    virtual void presetup(double fraction) override;
    void rotate(double angle, const Vector3d& axis) const override;
    void translate(const Vector3d& translation_vector) const override;
    double get_mass() const override;
    void reset_loads() override;

    /**
     * @brief Returns the RPM of the rotor [rpm]
     */
    double get_rpm() const;

    /**
     * @brief Returns the axial torque of the rotor [Nm]
     */
    double get_axial_torque() const;

    /**
     * @brief Returns the lateral torque of the rotor (y component) [Nm]
     */
    double get_lateral_torque() const;

    /**
     * @brief Returns the axial thrust of the rotor [N]
     */
    double get_axial_thrust() const;

    /**
     * @brief Returns azimuth of rotor [rad]
     */
    double get_azimuth() const;

    /**
     * @brief Accumulates torque on the rotor.
     *
     * @param[in] torque Torque to accumulate on axial axis of hub [Nm]
     */
    void accumulate_electrical_torque(double torque);

    /**
     * @brief Returns the electrical torque applied on the rotor [Nm]
     */
    double get_electrical_torque() const;

    /**
     * @brief Applies yaw increment to the RNA, rotating it around its mounting points.
     *
     * @param yaw_increment Yaw increment value [rad]
     */
    void apply_yaw_increment(double yaw_increment);

    /**
     * @brief Returns current yaw of the RNA [rad]
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
    /** @brief Accumulated electrical torque on the rotor [Nm] */
    double torque_elec_accumulated = 0.0;
    /** @brief Whether the RNA is mounted (such as on a tower) or not. */
    bool is_mounted = false;
};

}  // namespace elasto
}  // namespace seahowl
