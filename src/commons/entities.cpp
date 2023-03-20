#include "seahowl/commons/entities.h"

using namespace seahowl;

void EntityEigen::set_position(Vector3d position) {
    this->position = position;
}

Vector3d EntityEigen::get_position() const {
    return position;
}

void EntityEigen::set_rotation(Quaternion rotation) {
    this->rotation = rotation;
}

Quaternion EntityEigen::get_rotation() const {
    return rotation;
}

void EntityDynamicEigen::set_velocity(Vector3d velocity) {
    this->velocity = velocity;
}

Vector3d EntityDynamicEigen::get_velocity() const {
    return velocity;
}

void EntityDynamicEigen::set_acceleration(Vector3d acceleration) {
    this->acceleration = acceleration;
}

Vector3d EntityDynamicEigen::get_acceleration() const {
    return acceleration;
}

void EntityDynamicEigen::set_rotational_velocity_local(Vector3d rotational_velocity_local) {
    this->rotational_velocity_local = rotational_velocity_local;
}

Vector3d EntityDynamicEigen::get_rotational_velocity_local() const {
    return rotational_velocity_local;
}

void EntityDynamicEigen::set_rotational_acceleration_local(Vector3d rotational_acceleration_local) {
    this->rotational_acceleration_local = rotational_acceleration_local;
}

Vector3d EntityDynamicEigen::get_rotational_acceleration_local() const {
    return rotational_acceleration_local;
}

void EntityDynamicEigen::set_rotational_velocity_global(Vector3d rotational_velocity_global) {
    this->rotational_velocity_global = rotational_velocity_global;
}

Vector3d EntityDynamicEigen::get_rotational_velocity_global() const {
    return rotational_velocity_global;
}

void EntityDynamicEigen::set_rotational_acceleration_global(Vector3d rotational_acceleration_global) {
    this->rotational_acceleration_global = rotational_acceleration_global;
}

Vector3d EntityDynamicEigen::get_rotational_acceleration_global() const {
    return rotational_acceleration_global;
}
