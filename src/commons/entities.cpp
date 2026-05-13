// SPDX-License-Identifier: Apache-2.0
// Copyright 2022–2026 TotalEnergies
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//

#include "seahowl/commons/entities.h"

// SEAHOWL headers
#include "seahowl/commons/numerics.h"

using namespace seahowl;

Vector3d Entity::get_rpy_angles() const {
    auto rot = get_rotation();
    auto qw = rot.w();
    auto qx = rot.x();
    auto qy = rot.y();
    auto qz = rot.z();
    // roll
    auto roll = std::atan2(2 * (qw * qx + qy * qz), 1 - 2 * (qx * qx + qy * qy));
    // pitch
    auto pitch = 2 * std::atan2(std::sqrt(1 + 2 * (qw * qy - qx * qz)), std::sqrt(1 - 2 * (qw * qy - qx * qz))) -
                 seahowl::PI / 2;
    // yaw
    auto yaw = std::atan2(2 * (qw * qz + qx * qy), 1 - 2 * (qy * qy + qz * qz));

    return seahowl::Vector3d(roll, pitch, yaw);
}

void Entity::rotate(double angle, const Vector3d& axis) {
    auto rotation = AngleAxisd(angle, axis);
    auto new_position = rotation * get_position();
    auto new_rotation = (rotation * get_rotation()).normalized();
    set_position(new_position);
    set_rotation(new_rotation);
}

void Entity::translate(const Vector3d& translation_vector) {
    set_position(get_position() + translation_vector);
}

void EntityDynamicEigen::set_position(const Vector3d& position) {
    this->position = position;
}

Vector3d EntityDynamicEigen::get_position() const {
    return position;
}

void EntityDynamicEigen::set_rotation(const Quaternion& rotation) {
    this->rotation = rotation;
}

Quaternion EntityDynamicEigen::get_rotation() const {
    return rotation;
}

void EntityDynamicEigen::set_velocity(const Vector3d& velocity) {
    this->velocity = velocity;
}

Vector3d EntityDynamicEigen::get_velocity() const {
    return velocity;
}

void EntityDynamicEigen::set_acceleration(const Vector3d& acceleration) {
    this->acceleration = acceleration;
}

Vector3d EntityDynamicEigen::get_acceleration() const {
    return acceleration;
}

void EntityDynamicEigen::set_rotational_velocity(const Vector3d& rotational_velocity, bool is_local) {
    if (is_local) {
        this->rotational_velocity = rotational_velocity;
    } else {
        this->rotational_velocity = get_rotation().inverse() * rotational_velocity;
    }
}

Vector3d EntityDynamicEigen::get_rotational_velocity(bool is_local) const {
    if (is_local) {
        return rotational_velocity;
    } else {
        return get_rotation() * rotational_velocity;
    }
}

void EntityDynamicEigen::set_rotational_acceleration(const Vector3d& rotational_acceleration, bool is_local) {
    if (is_local) {
        this->rotational_acceleration = rotational_acceleration;
    } else {
        this->rotational_acceleration = get_rotation().inverse() * rotational_acceleration;
    }
}

Vector3d EntityDynamicEigen::get_rotational_acceleration(bool is_local) const {
    if (is_local) {
        return rotational_acceleration;
    } else {
        return get_rotation() * rotational_acceleration;
    }
}
