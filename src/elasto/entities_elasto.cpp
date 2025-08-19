#include "seahowl/elasto/entities_elasto.h"

using namespace seahowl;
using namespace seahowl::elasto;

Vector3d EntityLoadable::get_force_total(bool is_local) const {
    return get_force(is_local) + get_force_internals(is_local);
}

Vector3d EntityLoadable::get_torque_total(bool is_local) const {
    return get_torque(is_local) + get_torque_internals(is_local);
}

Vector3d ElementElasto::get_position(double eta) const {
    auto position = Vector3d(0.0, 0.0, 0.0);
    auto rotation = Quaternion(0.0, 0.0, 0.0, 0.0);
    evaluate_position_rotation(eta, position, rotation);
    return position;
}

Quaternion ElementElasto::get_rotation(double eta) const {
    auto position = Vector3d(0.0, 0.0, 0.0);
    auto rotation = Quaternion(0.0, 0.0, 0.0, 0.0);
    evaluate_position_rotation(eta, position, rotation);
    return rotation;
}

Vector3d ElementElasto::get_force(double eta) const {
    auto force = Vector3d(0.0, 0.0, 0.0);
    auto torque = Vector3d(0.0, 0.0, 0.0);
    evaluate_force_torque(eta, force, torque);
    return force;
}

Vector3d ElementElasto::get_torque(double eta) const {
    auto force = Vector3d(0.0, 0.0, 0.0);
    auto torque = Vector3d(0.0, 0.0, 0.0);
    evaluate_force_torque(eta, force, torque);
    return torque;
}

Vector3d ActuatorRotation::get_rotation_axis() const {
    return body_controller->get_rotation() * (reference_rotation * Vector3d(0.0, 0.0, 1.0));
}

void ActuatorRotation::set_position(const Vector3d& position) {
    body_worker->set_position(position);
    body_controller->set_position(position);
}

Vector3d ActuatorRotation::get_position() const {
    return body_controller->get_position();
}

void ActuatorRotation::set_rotation(const Quaternion& rotation) {
    auto rotation_relative = body_controller->get_rotation().inverse() * body_worker->get_rotation();
    body_controller->set_rotation(rotation);
    body_worker->set_rotation(rotation * rotation_relative);
}

Quaternion ActuatorRotation::get_rotation() const {
    return body_controller->get_rotation();
}

Vector3d ActuatorRotation::get_rpy_angles() const {
    return body_controller->get_rpy_angles();
}
