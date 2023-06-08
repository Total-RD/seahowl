#include "seahowl/commons/entities.h"

using namespace seahowl;

void EntityEigen::set_position(const Vector3d& position) {
    this->position = position;
}

Vector3d EntityEigen::get_position() const {
    return position;
}

void EntityEigen::set_rotation(const Quaternion& rotation) {
    this->rotation = rotation;
}

Quaternion EntityEigen::get_rotation() const {
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

void EntityDynamicEigen::set_rotational_velocity(const Vector3d& rotational_velocity) {
    this->rotational_velocity = rotational_velocity;
}

Vector3d EntityDynamicEigen::get_rotational_velocity() const {
    return rotational_velocity;
}

void EntityDynamicEigen::set_rotational_acceleration(const Vector3d& rotational_acceleration) {
    this->rotational_acceleration = rotational_acceleration;
}

Vector3d EntityDynamicEigen::get_rotational_acceleration() const {
    return rotational_acceleration;
}
