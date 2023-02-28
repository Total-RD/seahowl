#include <seahowl/elasto/rotor_elasto.h>

#include <seahowl/elasto/blade_elasto.h>
#include <seahowl/elasto/tower_elasto.h>

using seahowl::elasto::BladeElasto;
using seahowl::elasto::RotorElasto;
// using seahowl::elasto::TowerElasto;

RotorElasto::RotorElasto() {}

void RotorElasto::assemble(seahowl::elasto::SystemElasto& system) {
    system.Add(body_hub);
    system.Add(body_shaft);
    system.Add(link_shaft_hub);
    system.Add(body_nacelle);
    system.Add(link_shaft_nacelle);
    system.Add(body_yaw_bearing);
    system.Add(link_shaft_yaw_bearing);
    for (auto link_blade : links_blades) {
        system.Add(link_blade);
    }
}

void RotorElasto::build(std::vector<std::shared_ptr<BladeElasto>> blades) {
    this->blades = blades;

    auto rotation0 = Quaternion(1.0, 0.0, 0.0, 0.0);

    // hub
    body_hub = std::make_shared<RigidBody>();
    // move hub along X for overhang and COG offset, and along Z for distance from towertop
    body_hub->set_position(Vector3d(hub.overhang + hub.center_of_mass, 0.0, 0.0));
    // local Z axis along global X axis + shaft tilt along global Y axis
    auto tilt_hub = Quaternion(Q_from_AngAxis(shaft.tilt, -chrono::VECT_Y));
    body_hub->set_rotation(tilt_hub * Q_from_AngAxis(PI / 2.0, chrono::VECT_Y));
    body_hub->set_position((tilt_hub * body_hub->GetPos()) + Vector3d(0.0, 0.0, shaft.distance_from_towertop));
    // mass and inertia
    body_hub->set_mass(hub.mass);
    body_hub->SetInertiaXX(Vector3d(0., 0., hub.inertia));

    // shaft
    body_shaft = std::make_shared<RigidBody>();
    // move end of shaft at yaw axis of nacelle
    body_shaft->set_position(Vector3d(0.0, 0.0, shaft.distance_from_towertop));
    // align rotation
    body_shaft->set_rotation(body_hub->GetRot());
    // massless body
    body_shaft->set_mass(0.0);
    // link hub to shaft
    link_shaft_hub = std::make_shared<LinkRevolute>();
    link_shaft_hub->initialize(body_hub, body_shaft);

    // nacelle
    body_nacelle = std::make_shared<RigidBody>();
    body_nacelle->set_position(nacelle.center_of_mass);
    body_nacelle->set_rotation(rotation0);
    // mass and inertia
    body_nacelle->set_mass(nacelle.mass);
    ///@todo  change to full 3x3 inertia matrix
    body_nacelle->SetInertiaXX(Vector3d(0.0, 0.0, nacelle.inertia));
    // link nacelle body to shaft body
    link_shaft_nacelle = std::make_shared<LinkFix>();
    link_shaft_nacelle->initialize(body_nacelle, body_shaft);

    // yaw bearing
    body_yaw_bearing = std::make_shared<RigidBody>();
    body_yaw_bearing->set_position(Vector3d(0.0, 0.0, 0.0));
    body_yaw_bearing->set_rotation(rotation0);
    body_yaw_bearing->set_mass(nacelle.yaw_bearing_mass);
    // link yaw bearing body to shaft body
    // link_shaft_yaw_bearing = chrono_types::make_shared<ChLinkRevolute>();
    link_shaft_yaw_bearing = std::make_shared<LinkFix>();
    link_shaft_yaw_bearing->initialize(body_shaft, body_yaw_bearing);

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
        blade->rotate(precone, chrono::VECT_Y);  // Y is the edge-wise axis for blade (IEC standard)
        double azimuth0 = ii * chrono::CH_C_2PI / nblades;
        blade->azimuth0 = azimuth0;
        // add overhang
        blade->translate(Vector3d(hub.overhang, 0.0, 0.0));
        // rotate blade around hub
        blade->rotate(azimuth0, chrono::VECT_X);  // X is the axis pointing towards nacelle for blade (IEC standard)
        // apply shaft tilt to blades
        blade->rotate(shaft.tilt, -chrono::VECT_Y);
        // offset with distance from towertop
        blade->translate(Vector3d(0.0, 0.0, shaft.distance_from_towertop));

        // link root node of blade to rotor center
        auto link_hub_blade = chrono_types::make_shared<LinkFix>();
        link_hub_blade->initialize(blade->nodes[0], body_hub);
        links_blades.push_back(link_hub_blade);
    }
}

void RotorElasto::link_tower(const TowerElasto& tower, seahowl::elasto::SystemElasto& system) {
    auto towertop_node = tower.nodes[tower.nodes.size() - 1];
    // translate RNA center of origin to towertop
    this->translate(towertop_node->GetPos());
    // link yaw bearing body to towertop
    link_towertop_yaw_bearing = chrono_types::make_shared<LinkFix>();
    system.Add(link_towertop_yaw_bearing);
    link_towertop_yaw_bearing->initialize(towertop_node, body_yaw_bearing);
}

void RotorElasto::rotate(double angle, const Vector3d& axis) const {
    // blades
    for (auto& blade : blades) {
        blade->rotate(angle, axis);
    }
    auto rotation = Quaternion(Q_from_AngAxis(angle, axis));
    // hub
    auto new_position_hub = rotation * body_hub->get_position();
    auto new_rotation_hub = (rotation * body_hub->get_rotation()).normalized();
    body_hub->SetPos(new_position_hub);
    body_hub->SetRot(new_rotation_hub);
    // shaft
    auto new_position_shaft = rotation * body_shaft->GetPos();
    auto new_rotation_shaft = (rotation * body_shaft->get_rotation()).normalized();
    body_shaft->SetPos(new_position_shaft);
    body_shaft->SetRot(new_rotation_shaft);
    // nacelle
    auto new_position_nacelle = rotation * body_nacelle->get_position();
    auto new_rotation_nacelle = (rotation * body_nacelle->get_rotation()).normalized();
    body_nacelle->SetPos(new_position_nacelle);
    body_nacelle->SetRot(new_rotation_nacelle);
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
        auto link = links_blades[ii];
        link->Initialize(blade->nodes.front(), body_hub);
    }
    pitch_collective += pitch_increment;
}

double RotorElasto::get_rpm() const {
    Vector3d angles;
    body_hub->coord.rot.Qdt_to_Wrel(angles, body_hub->coord_dt.rot);
    double rpm = -angles.z() * 60 / (2 * PI);
    return rpm;
}

double RotorElasto::get_azimuth() const {
    auto rotation_relative = body_shaft->GetCoord().TransformParentToLocal(body_hub->GetCoord()).rot.Q_to_Euler123();
    double angle = rotation_relative.z();
    return angle;
}

double RotorElasto::get_axial_thrust() const {
    auto react_force = link_shaft_hub->Get_react_force();
    return react_force.z();
}

double RotorElasto::get_axial_torque() const {
    auto react_torque = link_shaft_hub->Get_react_torque();
    return react_torque.x();
}
