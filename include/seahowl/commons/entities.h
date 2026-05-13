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

// Disable inherits via dominance warning when there is multiple inheritance
#pragma warning(disable : 4250)

// SEAHOWL headers
#include "seahowl/commons/numerics.h"

namespace seahowl {

/**
 * @brief Base entity, with position and rotation.
 */
class Entity {
  public:
    /**
     * @brief Virtual destructor.
     */
    virtual ~Entity() = default;
    /**
     * @brief Sets position of entity.
     *
     * @param[in] position Position of entity [m]
     */
    virtual void set_position(const Vector3d& position) = 0;

    /**
     * @brief Returns position of entity [m]
     */
    virtual Vector3d get_position() const = 0;

    /**
     * @brief Sets rotation of entity.
     *
     * @param[in] rotation Rotation of entity.
     */
    virtual void set_rotation(const Quaternion& rotation) = 0;

    /**
     * @brief Returns rotation of entity.
     */
    virtual Quaternion get_rotation() const = 0;

    /**
     * @brief Returns RPY (roll-pitch-yaw) angles of entity [rad]
     */
    virtual Vector3d get_rpy_angles() const;

    /**
     * @brief Returns direction of entity (local Z-axis projected in global frame).
     */
    virtual Vector3d get_direction() const { return get_rotation() * Vector3d(0.0, 0.0, 1.0); };

    /**
     * @brief Returns rotation matrix of entity.
     */
    Eigen::Matrix<double, 3, 3> get_rotation_matrix() const { return get_rotation().toRotationMatrix(); };

    /**
     * @brief Sets rotation matrix of entity.
     *
     * @param[in] rotation Rotation matrix of entity.
     */
    void set_rotation_matrix(Eigen::Matrix<double, 3, 3> rotation) { set_rotation(Quaternion(rotation)); };

    /**
     * @brief Rotates the entity.
     *
     * @param[in] angle The angle of rotation [rad]
     * @param[in] axis The axis of rotation (3D vector).
     */
    void rotate(double angle, const Vector3d& axis);

    /**
     * @brief Translates the entity.
     *
     * @param[in] translation_vector The 3D translation vector.
     */
    void translate(const Vector3d& translation_vector);
};

/**
 * @brief Base dynamic entity, with position, rotation, velocity, acceleration.
 */
class EntityDynamic : public virtual Entity {
  public:
    /**
     * @brief Sets velocity of entity.
     *
     * @param[in] velocity Velocity of entity [m/s]
     */
    virtual void set_velocity(const Vector3d& velocity) = 0;

    /**
     * @brief Returns rotation of entity.
     */
    virtual Vector3d get_velocity() const = 0;

    /**
     * @brief Sets acceleration of entity.
     *
     * @param[in] acceleration Acceleration of entity [m/s^2]
     */
    virtual void set_acceleration(const Vector3d& acceleration) = 0;

    /**
     * @brief Returns acceleration of entity [m/s^2]
     */
    virtual Vector3d get_acceleration() const = 0;

    /**
     * @brief Sets rotational velocity of entity (global reference frame).
     *
     * @param[in] rotational_velocity_global of entity [rad/s]
     */
    virtual void set_rotational_velocity(const Vector3d& rotational_velocity, bool is_local = true) = 0;

    /**
     * @brief Returns rotational velocity of entity (global reference frame) [rad/s]
     */
    virtual Vector3d get_rotational_velocity(bool is_local = true) const = 0;

    /**
     * @brief Sets rotational acceleration of entity (global reference frame).
     *
     * @param[in] rotational_acceleration_global of entity [rad/s^2]
     */
    virtual void set_rotational_acceleration(const Vector3d& rotational_acceleration, bool is_local = true) = 0;

    /**
     * @brief Returns rotational acceleration of entity (global reference frame) [rad/s^2]
     */
    virtual Vector3d get_rotational_acceleration(bool is_local = true) const = 0;

    /**
     * @brief Ensure a virtual destructor.
     */
    virtual ~EntityDynamic() = default;
};

/**
 * @brief Dynamic entity implemented with Eigen types.
 */
class EntityDynamicEigen : public EntityDynamic {
  protected:
    /** @brief Position of entity [m] */
    Vector3d position{0.0, 0.0, 0.0};
    /** @brief Rotation of entity. */
    Quaternion rotation{0.0, 0.0, 0.0, 0.0};
    /** @brief Velocity of entity [m/s] */
    Vector3d velocity{0.0, 0.0, 0.0};
    /** @brief Acceleration of entity [m/s^2] */
    Vector3d acceleration{0.0, 0.0, 0.0};
    /** @brief Rotational velocity of entity (in global reference frame) [rad/s] */
    Vector3d rotational_velocity{0.0, 0.0, 0.0};
    /** @brief Rotational acceleration of entity (in global reference frame) [rad/s^2] */
    Vector3d rotational_acceleration{0.0, 0.0, 0.0};

  public:
    virtual void set_position(const Vector3d& position) override;
    virtual Vector3d get_position() const override;
    virtual void set_rotation(const Quaternion& rotation) override;
    virtual Quaternion get_rotation() const override;
    virtual void set_velocity(const Vector3d& velocity) override;
    virtual Vector3d get_velocity() const override;
    virtual void set_acceleration(const Vector3d& acceleration) override;
    virtual Vector3d get_acceleration() const override;
    virtual void set_rotational_velocity(const Vector3d& rotational_velocity, bool is_local = true) override;
    virtual Vector3d get_rotational_velocity(bool is_local = true) const override;
    virtual void set_rotational_acceleration(const Vector3d& rotational_acceleration, bool is_local = true) override;
    virtual Vector3d get_rotational_acceleration(bool is_local = true) const override;
};

}  // namespace seahowl
