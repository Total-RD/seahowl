#include "seahowl/elasto/floater_elasto.h"

#include "seahowl/elasto/chrono_adapters.h"

using namespace seahowl::elasto;

FloaterElastoRigid::FloaterElastoRigid() {
    floater_body = std::make_unique<seahowl::elasto::BodyElastoChrono>();
};

void FloaterElastoRigid::add_fairlead(Vector3d& position) {
    // fairlead
    fairleads.push_back(std::unique_ptr<seahowl::elasto::BodyElastoChrono>());
    auto& fairlead = *(fairleads.back());
    fairlead.set_position(position);

    // link
    links_fairlead_floater.push_back(std::unique_ptr<seahowl::elasto::LinkChrono>());
    auto& link = *(links_fairlead_floater.back().get());
    link.set_constraints(true, true, true, false, false, false);
    link.initialize(fairlead, *floater_body);
}

void FloaterElastoRigid::assemble(seahowl::elasto::SystemElasto& system) {
    system.add(*floater_body);
}

void FloaterElastoRigid::initialize(double time, double dt) {}

seahowl::elasto::BodyElasto& FloaterElastoRigid::get_tower_connection_body() {
    return *floater_body;
}

void FloaterElastoRigid::translate(Vector3d translation_vector) {
    floater_body->set_position(floater_body->get_position() + translation_vector);
}

void FloaterElastoRigid::rotate(double angle, Vector3d axis) {
    auto rotation = AngleAxisd(angle, axis);
    // rotate body
    auto new_position_body = rotation * floater_body->get_position();
    auto new_rotation_body = (rotation * floater_body->get_rotation()).normalized();
    floater_body->set_position(new_position_body);
    floater_body->set_rotation(new_rotation_body);
}
