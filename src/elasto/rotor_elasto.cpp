#include "seahowl/elasto/rotor_elasto.h"

#include "seahowl/commons/numerics.h"
#include <seahowl/elasto/blade_elasto.h>
#include <seahowl/elasto/chrono_adapters.h>

using seahowl::elasto::BladeElasto;
using seahowl::elasto::RotorElasto;

RotorElasto::RotorElasto() {}

void RotorElasto::assemble(SystemElasto& system) {
    for (auto& blade : blades) {
        blade->assemble(system);
    }
    system.add(*(body_hub.get()));
    system.add(*(body_shaft.get()));
    system.add(*(link_shaft_hub.get()));
    system.add(*(body_nacelle.get()));
    system.add(*(link_shaft_nacelle.get()));
    system.add(*(body_yaw_bearing.get()));
    system.add(*(link_shaft_yaw_bearing.get()));
    for (auto& link_blade : links_blades) {
        system.add(*(link_blade.get()));
    }
}

void RotorElasto::build() {
    // build blades
    for (auto& blade : blades) {
        blade->build();
    }

    auto rotation0 = Quaternion(1.0, 0.0, 0.0, 0.0);

    // hub
    body_hub = std::make_unique<BodyElastoChrono>();
    // move hub along X for overhang and COG offset, and along Z for distance from towertop
    body_hub->set_position(Vector3d(hub.overhang + hub.center_of_mass, 0.0, 0.0));
    // local Z axis along global X axis + shaft tilt along global Y axis
    auto tilt_hub = Quaternion(AngleAxisd(shaft.tilt, -Vector3d(0.0, 1.0, 0.0)));
    body_hub->set_rotation(tilt_hub);
    body_hub->set_position((tilt_hub * body_hub->get_position()) + Vector3d(0.0, 0.0, shaft.distance_from_towertop));
    // mass and inertia
    body_hub->set_mass(hub.mass);
    body_hub->set_inertia_diagonal(Vector3d(0., 0., hub.inertia));

    // shaft
    body_shaft = std::make_unique<BodyElastoChrono>();
    // move end of shaft at yaw axis of nacelle
    body_shaft->set_position(Vector3d(0.0, 0.0, shaft.distance_from_towertop));
    // align rotation
    body_shaft->set_rotation(body_hub->get_rotation());
    // massless body
    body_shaft->set_mass(0.0);
    // link hub to shaft
    link_shaft_hub = std::make_unique<LinkChrono>();
    link_shaft_hub->initialize(*(body_hub.get()), *(body_shaft.get()));
    link_shaft_hub->set_constraints(true, true, true, false, true, true);

    // nacelle
    body_nacelle = std::make_unique<BodyElastoChrono>();
    body_nacelle->set_position(nacelle.center_of_mass);
    body_nacelle->set_rotation(rotation0);
    // mass and inertia
    body_nacelle->set_mass(nacelle.mass);
    ///@todo  change to full 3x3 inertia matrix
    body_nacelle->set_inertia_diagonal(Vector3d(0.0, 0.0, nacelle.inertia));
    // link nacelle body to shaft body
    link_shaft_nacelle = std::make_unique<LinkChrono>();
    link_shaft_nacelle->initialize(*(body_nacelle.get()), *(body_shaft.get()));
    link_shaft_nacelle->set_constraints(true, true, true, true, true, true);

    // yaw bearing
    body_yaw_bearing = std::make_unique<BodyElastoChrono>();
    body_yaw_bearing->set_position(Vector3d(0.0, 0.0, 0.0));
    body_yaw_bearing->set_rotation(rotation0);
    body_yaw_bearing->set_mass(nacelle.yaw_bearing_mass);
    // link yaw bearing body to shaft body
    link_shaft_yaw_bearing = std::make_unique<LinkChrono>();
    link_shaft_yaw_bearing->initialize(*(body_shaft.get()), *(body_yaw_bearing.get()));
    link_shaft_yaw_bearing->set_constraints(true, true, true, true, true, true);

    // blades
    links_blades.clear();
    auto nblades = blades.size();
    for (int ii = 0; ii < nblades; ii++) {
        auto blade = blades[ii];
        double precone = blade_precones[ii];

        // rotations + translations
        // blade root node is assumed to be originally at (0,0,0) and using IEC standard for coordinate system
        // offset blade from hub apex
        blade->translate(Vector3d(0.0, 0.0, hub.radius));
        // apply precone
        blade->rotate(precone, Vector3d(0.0, 1.0, 0.0));  // Y is the edge-wise axis for blade (IEC standard)
        double azimuth0 = ii * 2 * PI / nblades;
        blade->azimuth0 = azimuth0;
        // add overhang
        blade->translate(Vector3d(hub.overhang, 0.0, 0.0));
        // rotate blade around hub
        blade->rotate(azimuth0,
                      Vector3d(1.0, 0.0, 0.0));  // X is the axis pointing towards nacelle for blade (IEC standard)
        // apply shaft tilt to blades
        blade->rotate(shaft.tilt, Vector3d(0.0, -1.0, 0.0));
        // offset with distance from towertop
        blade->translate(Vector3d(0.0, 0.0, shaft.distance_from_towertop));

        // link root node of blade to rotor center
        links_blades.push_back(std::make_unique<LinkChrono>());
        auto& link_hub_blade = links_blades.back();
        link_hub_blade->initialize(*(blade->nodes[0].get()), *(body_hub.get()));
        link_hub_blade->set_constraints(true, true, true, true, true, true);
    }
}

void RotorElasto::rotate(double angle, const Vector3d& axis) const {
    // blades
    for (auto& blade : blades) {
        blade->rotate(angle, axis);
    }
    auto rotation = AngleAxisd(angle, axis);
    // hub
    auto new_position_hub = rotation * body_hub->get_position();
    auto new_rotation_hub = (rotation * body_hub->get_rotation()).normalized();
    body_hub->set_position(new_position_hub);
    body_hub->set_rotation(new_rotation_hub);
    // shaft
    auto new_position_shaft = rotation * body_shaft->get_position();
    auto new_rotation_shaft = (rotation * body_shaft->get_rotation()).normalized();
    body_shaft->set_position(new_position_shaft);
    body_shaft->set_rotation(new_rotation_shaft);
    // nacelle
    auto new_position_nacelle = rotation * body_nacelle->get_position();
    auto new_rotation_nacelle = (rotation * body_nacelle->get_rotation()).normalized();
    body_nacelle->set_position(new_position_nacelle);
    body_nacelle->set_rotation(new_rotation_nacelle);
    // yaw bearing
    auto new_position_yaw_bearing = rotation * body_yaw_bearing->get_position();
    auto new_rotation_yaw_bearing = (rotation * body_yaw_bearing->get_rotation()).normalized();
    body_yaw_bearing->set_position(new_position_yaw_bearing);
    body_yaw_bearing->set_rotation(new_rotation_yaw_bearing);
}

void RotorElasto::translate(const Vector3d& translation_vector) const {
    // blades
    for (auto& blade : blades) {
        blade->translate(translation_vector);
    }
    // hub
    body_hub->set_position(body_hub->get_position() + translation_vector);
    // shaft
    body_shaft->set_position(body_shaft->get_position() + translation_vector);
    // nacelle
    body_nacelle->set_position(body_nacelle->get_position() + translation_vector);
    // yaw_bearing
    body_yaw_bearing->set_position(body_yaw_bearing->get_position() + translation_vector);
}

double RotorElasto::get_mass() const {
    double total_mass = 0.0;
    // blades
    for (auto& blade : blades) {
        total_mass += blade->get_mass();
    }
    // hub
    total_mass += body_hub->get_mass();
    // shaft
    total_mass += body_shaft->get_mass();
    // nacelle
    total_mass += body_nacelle->get_mass();
    // yaw_bearing
    total_mass += body_yaw_bearing->get_mass();
    return total_mass;
}

void RotorElasto::apply_collective_pitch_increment(double pitch_increment) {
    for (int ii = 0; ii < blades.size(); ii++) {
        // apply pitch on blade
        auto blade = blades[ii];
        blade->apply_pitch_increment(pitch_increment);
        // update blade-hub constraint
        auto& link = links_blades[ii];
        link->initialize(*(blade->nodes.front().get()), *(body_hub.get()));
    }
    pitch_collective += pitch_increment;
}

double RotorElasto::get_rpm() const {
    // relative rotational velocity between hub and shaft (in local reference frame of the hub)
    auto rotational_velocity = body_hub->get_rotation().inverse() *
                               (body_hub->get_rotational_velocity() - body_shaft->get_rotational_velocity());
    // convert to rpm
    auto rpm = rotational_velocity.x() * 60 / (2 * PI);
    return rpm;
}

double RotorElasto::get_azimuth() const {
    // get angle between quaternions
    auto qq = (body_shaft->get_rotation().conjugate() * body_hub->get_rotation()).normalized();
    double angle0 = 2 * std::atan2(qq.vec().x(), qq.w());
    // get angle between 0 and 2pi
    double angle1 = fmod(angle0, 2 * PI);
    // get strictly positive angle
    double angle2 = fmod(angle1 + 2 * PI, 2 * PI);
    return angle2;
}

double RotorElasto::get_axial_thrust() const {
    auto react_force = link_shaft_hub->get_reaction_force();
    return react_force.x();
}

double RotorElasto::get_axial_torque() const {
    // get reaction torque from all blades linked to hub
    // those links are already in the hub body reference frame
    auto react_torque = Vector3d(0.0, 0.0, 0.0);
    for (auto& link_blade : links_blades) {
        react_torque += link_blade->get_reaction_torque();
    }
    return react_torque.x();
}

void RotorElasto::accumulate_axial_torque(double torque) {
    body_hub->accumulate_torque(Vector3d(torque, 0.0, 0.0), true);
}
