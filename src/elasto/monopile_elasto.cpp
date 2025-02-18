#include "seahowl/elasto/monopile_elasto.h"

using namespace seahowl;
using namespace seahowl::elasto;

MonopileElasto::MonopileElasto() : TowerElasto() {
    body_tp = std::make_unique<BodyElastoChrono>();
    body_tp->set_mass(0.0);
    body_tp->set_inertia_diagonal(Vector3d(0.0, 0.0, 0.0));
    link_tp_entity = std::make_unique<seahowl::elasto::LinkChrono>();
    link_tp_monopile = std::make_unique<seahowl::elasto::LinkChrono>();
}

void MonopileElasto::link_to_entity(const Entity& entity) {
    // move TP to monopile top
    link_tp_entity->initialize(*body_tp, entity);
    link_tp_entity->set_constraints(true, true, true, true, true, true);
    is_linked = true;
}

void MonopileElasto::set_fixed(bool is_fixed) {
    nodes.front()->set_fixed(is_fixed);
}

bool MonopileElasto::is_fixed() const {
    return nodes.front()->is_fixed();
}

void MonopileElasto::build() {
    // first build nodes
    TowerElasto::build();
    // fix bottom of monopile
    set_fixed(true);
    // move TP to top of monopile
    body_tp->set_position(nodes.back()->get_position());
    link_tp_monopile->initialize(*body_tp, *nodes.back());
    link_tp_monopile->set_constraints(true, true, true, true, true, true);
}

void MonopileElasto::assemble_this(seahowl::elasto::SystemElasto& system) {
    TowerElasto::assemble_this(system);
    system.add(*link_tp_monopile);
    system.add(*body_tp);
    if (is_linked) {
        system.add(*link_tp_entity);
    }
}

void MonopileElasto::translate(const Vector3d& translation_vector) const {
    TowerElasto::translate(translation_vector);
    body_tp->translate(translation_vector);
}

void MonopileElasto::rotate(double angle, const Vector3d& axis) const {
    TowerElasto::rotate(angle, axis);
    body_tp->rotate(angle, axis);
}

double MonopileElasto::get_mass() const {
    double total_mass = 0;
    total_mass += body_tp->get_mass();
    total_mass += TowerElasto::get_mass();
    return total_mass;
}
