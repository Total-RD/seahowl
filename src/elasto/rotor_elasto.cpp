#include "seahowl/elasto/rotor_elasto.h"

#include "seahowl/commons/numerics.h"
#include "seahowl/elasto/blade_elasto.h"
#include "seahowl/elasto/chrono_adapters.h"

using seahowl::elasto::BladeElasto;
using seahowl::elasto::RotorElasto;
using seahowl::elasto::RotorNacelleAssemblyElasto;

RotorElasto::RotorElasto() {}

void RotorElasto::assemble_this(SystemElasto& system) {
    for (auto& blade : blades) {
        blade->assemble(system);
    }
    system.add(*body_hub);
}

void RotorElasto::presetup(double fraction) {
    for (auto& blade : blades) {
        blade->presetup(fraction);
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
    // mass and inertia
    body_hub->set_mass(hub.mass);
    body_hub->set_inertia_matrix(hub.inertia);
    body_hub->set_position(hub.position_from_apex);
    body_hub->set_rotation(rotation0);

    // blades
    auto nblades = blades.size();
    for (int ii = 0; ii < nblades; ii++) {
        auto blade = blades[ii];

        // rotations + translations
        // blade root node is assumed to be originally at (0,0,0) and using IEC standard for coordinate system
        // offset blade from hub apex
        blade->translate(Vector3d(0.0, 0.0, hub.radius));
        // apply precone
        blade->rotate(blade->precone, Vector3d(0.0, 1.0, 0.0));  // Y is the edge-wise axis for blade (IEC standard)
        double azimuth0 = ii * 2 * PI / nblades;
        blade->azimuth0 = azimuth0;
        // rotate blade around hub
        blade->rotate(azimuth0,
                      Vector3d(1.0, 0.0, 0.0));  // X is the axis pointing towards nacelle for blade (IEC standard)

        // update blade-hub constraint
        blade->attach_blade_to_body(*body_hub);
    }
}

void RotorElasto::apply_collective_pitch_increment(double pitch_increment) {
    for (auto& blade : blades) {
        blade->apply_pitch_increment(pitch_increment);
    }
    pitch_collective += pitch_increment;
}

void RotorElasto::rotate(double angle, const Vector3d& axis) const {
    // blades
    for (auto& blade : blades) {
        blade->rotate(angle, axis);
    }
    // hub
    body_hub->rotate(angle, axis);
}

void RotorElasto::translate(const Vector3d& translation_vector) const {
    // blades
    for (auto& blade : blades) {
        blade->translate(translation_vector);
    }
    // hub
    body_hub->translate(translation_vector);
}

double RotorElasto::get_mass() const {
    double total_mass = 0.0;
    // blades
    for (auto& blade : blades) {
        total_mass += blade->get_mass();
    }
    // hub
    total_mass += body_hub->get_mass();
    return total_mass;
}

void RotorElasto::reset_loads() {
    body_hub->reset_loads();
}

void RotorElasto::accumulate_axial_torque(double torque) {
    body_hub->accumulate_torque(Vector3d(torque, 0.0, 0.0), true);
}

RotorNacelleAssemblyElasto::RotorNacelleAssemblyElasto() {
    // rotor
    rotor = std::make_unique<RotorElasto>();

    // shaft
    body_shaft = std::make_unique<BodyElastoChrono>();
    // link between hub and shaft
    link_shaft_hub = std::make_unique<LinkChrono>();
    link_shaft_hub->set_constraints(true, true, true, false, true, true);

    // nacelle
    body_nacelle = std::make_unique<BodyElastoChrono>();
    // link between nacelle body and shaft body
    link_shaft_nacelle = std::make_unique<LinkChrono>();
    link_shaft_nacelle->set_constraints(true, true, true, true, true, true);

    // yaw bearing
    body_yaw_bearing = std::make_unique<BodyElastoChrono>();
    // link between yaw bearing body and shaft body
    link_shaft_yaw_bearing = std::make_unique<LinkChrono>();
    link_shaft_yaw_bearing->set_constraints(true, true, true, true, true, true);

    // mounting point
    body_mount = std::make_unique<BodyElastoChrono>();
    // link between yaw bearing body and shaft body
    link_yaw_bearing_mount = std::make_unique<LinkChrono>();
    link_yaw_bearing_mount->set_constraints(true, true, true, true, true, true);

    // RNA link
    link_towertop_mount = std::make_unique<LinkChrono>();
    link_towertop_mount->set_constraints(true, true, true, true, true, true);
}

void RotorNacelleAssemblyElasto::presetup(double fraction) {
    rotor->presetup(fraction);
}

void RotorNacelleAssemblyElasto::assemble_this(SystemElasto& system) {
    rotor->assemble(system);
    system.add(*body_shaft);
    system.add(*link_shaft_hub);
    system.add(*body_nacelle);
    system.add(*link_shaft_nacelle);
    system.add(*body_yaw_bearing);
    system.add(*link_shaft_yaw_bearing);
    system.add(*body_mount);
    system.add(*link_yaw_bearing_mount);
    if (is_mounted) {
        system.add(*link_towertop_mount);
    }
}

void RotorNacelleAssemblyElasto::build() {
    // build blades
    rotor->build();

    // apply shaft tilt to blades
    rotor->translate(Vector3d(rotor->hub.overhang, 0.0, 0.0));
    rotor->rotate(shaft.tilt, Vector3d(0.0, -1.0, 0.0));
    rotor->translate(Vector3d(0.0, 0.0, shaft.distance_from_towertop));

    auto rotation0 = Quaternion(1.0, 0.0, 0.0, 0.0);

    // move end of shaft at yaw axis of nacelle
    body_shaft->set_position(Vector3d(0.0, 0.0, shaft.distance_from_towertop));
    // align rotation
    body_shaft->set_rotation(rotor->body_hub->get_rotation());
    // shaft
    body_shaft->set_mass(0.0);
    body_shaft->set_inertia_diagonal(Vector3d(0.0, 0.0, 0.0));
    link_shaft_hub->initialize(*rotor->body_hub, *body_shaft);

    // nacelle
    body_nacelle->set_position(nacelle.position_from_towertop);
    body_nacelle->set_rotation(rotation0);
    // mass and inertia
    body_nacelle->set_mass(nacelle.mass);
    ///@todo  change to full 3x3 inertia matrix
    body_nacelle->set_inertia_matrix(nacelle.inertia);
    // link nacelle body to shaft body
    link_shaft_nacelle->initialize(*body_nacelle, *body_shaft);

    // yaw bearing
    body_yaw_bearing->set_position(Vector3d(0.0, 0.0, 0.0));
    body_yaw_bearing->set_rotation(rotation0);
    body_yaw_bearing->set_mass(nacelle.yaw_bearing_mass);
    body_yaw_bearing->set_inertia_diagonal(Vector3d(0.0, 0.0, 0.0));
    // link yaw bearing body to shaft body
    link_shaft_yaw_bearing->initialize(*body_shaft, *body_yaw_bearing);

    // mounting point
    body_mount->set_position(Vector3d(0.0, 0.0, 0.0));
    body_mount->set_rotation(rotation0);
    body_mount->set_mass(0.0);
    body_mount->set_inertia_diagonal(Vector3d(0.0, 0.0, 0.0));
    // link mount to yaw bearing
    link_yaw_bearing_mount->initialize(*body_yaw_bearing, *body_mount);

    // apply initial yaw increment
    if (yaw0 != 0.0) {
        apply_yaw_increment(yaw0);
    }
}

void RotorNacelleAssemblyElasto::rotate(double angle, const Vector3d& axis) const {
    // rotor
    rotor->rotate(angle, axis);
    auto rotation = AngleAxisd(angle, axis);
    // shaft
    body_shaft->rotate(angle, axis);
    // nacelle
    body_nacelle->rotate(angle, axis);
    // yaw bearing
    body_yaw_bearing->rotate(angle, axis);
    // mounting point
    body_mount->rotate(angle, axis);
}

void RotorNacelleAssemblyElasto::translate(const Vector3d& translation_vector) const {
    // rotor
    rotor->translate(translation_vector);
    // shaft
    body_shaft->translate(translation_vector);
    // nacelle
    body_nacelle->translate(translation_vector);
    // yaw bearing
    body_yaw_bearing->translate(translation_vector);
    // mounting point
    body_mount->translate(translation_vector);
}

double RotorNacelleAssemblyElasto::get_mass() const {
    double total_mass = 0.0;
    // rotor
    total_mass += rotor->get_mass();
    // shaft
    total_mass += body_shaft->get_mass();
    // nacelle
    total_mass += body_nacelle->get_mass();
    // yaw bearing
    total_mass += body_yaw_bearing->get_mass();
    // mounting point
    total_mass += body_mount->get_mass();
    return total_mass;
}

void RotorNacelleAssemblyElasto::reset_loads() {
    rotor->reset_loads();
    body_shaft->reset_loads();
    torque_elec_accumulated = 0.0;
}

double RotorNacelleAssemblyElasto::get_rpm() const {
    // relative rotational velocity between hub and shaft (in local reference frame of the hub)
    auto rotational_velocity =
        (rotor->body_hub->get_rotational_velocity(true) - body_shaft->get_rotational_velocity(true));
    // convert to rpm
    auto rpm = rotational_velocity.x() * 60 / (2 * PI);
    return rpm;
}

double RotorNacelleAssemblyElasto::get_azimuth() const {
    // get angle between quaternions
    auto qq = (body_shaft->get_rotation().conjugate() * rotor->body_hub->get_rotation()).normalized();
    double angle0 = 2 * std::atan2(qq.vec().x(), qq.w());
    // get angle between 0 and 2pi
    double angle1 = fmod(angle0, 2 * PI);
    // get strictly positive angle
    double angle2 = fmod(angle1 + 2 * PI, 2 * PI);
    return angle2;
}

double RotorNacelleAssemblyElasto::get_axial_thrust() const {
    auto react_force = link_shaft_hub->get_reaction_force();
    return react_force.x();
}

double RotorNacelleAssemblyElasto::get_axial_torque() const {
    // get reaction torque from all blades linked to hub
    // those links are already in the hub body reference frame
    auto react_torque = Vector3d(0.0, 0.0, 0.0);
    for (auto& blade : rotor->blades) {
        react_torque += blade->link_blade->get_reaction_torque();
    }
    return react_torque.x();
}

void RotorNacelleAssemblyElasto::accumulate_electrical_torque(double torque) {
    rotor->body_hub->accumulate_torque(Vector3d(-torque, 0.0, 0.0), true);
    body_shaft->accumulate_torque(Vector3d(torque, 0.0, 0.0), true);
    torque_elec_accumulated += torque;
}

double RotorNacelleAssemblyElasto::get_electrical_torque() const {
    return torque_elec_accumulated;
}

void RotorNacelleAssemblyElasto::apply_yaw_increment(double yaw_increment) {
    // apply pitch from root node direction and position
    auto root_dir = body_mount->get_rotation() * Vector3d(0.0, 0.0, 1.0);
    auto root_pos = body_mount->get_position();
    translate(-root_pos);
    rotate(yaw_increment, root_dir);
    body_mount->rotate(-yaw_increment, root_dir);  // rotate mounting point back
    link_yaw_bearing_mount->initialize(*body_yaw_bearing, *body_mount);
    translate(root_pos);
}

double RotorNacelleAssemblyElasto::get_yaw() const {
    // get angle between quaternions
    auto qq = (body_yaw_bearing->get_rotation().conjugate() * body_mount->get_rotation()).normalized();
    double angle0 = 2 * std::atan2(qq.vec().z(), qq.w());
    // get angle between 0 and 2pi
    double angle1 = fmod(angle0, 2 * PI);
    return -angle1;
}

void RotorNacelleAssemblyElasto::set_fixed_yaw(bool is_fixed) {
    link_yaw_bearing_mount->set_constraints(true, true, true, true, true, is_fixed);
    link_yaw_bearing_mount->initialize(*body_yaw_bearing, *body_mount);
}

void RotorNacelleAssemblyElasto::attach_rna_to_body(const BodyElasto& body) {
    is_mounted = true;
    link_towertop_mount->initialize(*body_mount, body);
}

void RotorNacelleAssemblyElasto::attach_rna_to_node(const NodeElasto& node) {
    is_mounted = true;
    link_towertop_mount->initialize(*body_mount, node);
}
