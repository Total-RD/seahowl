#include "seahowl/elasto/foundation_elasto.h"

#include "seahowl/elasto/chrono_adapters.h"

using namespace seahowl::elasto;

FoundationElastoBody::FoundationElastoBody() {
    body_foundation = std::make_unique<BodyElastoChrono>();
    body_foundation->set_mass(0.0);
    body_foundation->set_inertia_diagonal(Vector3d(0.0, 0.0, 0.0));
    link_foundation_entity = std::make_unique<seahowl::elasto::LinkChrono>();
}

void FoundationElastoBody::link_to_entity(const Entity& entity) {
    // move body to entity position
    body_foundation->set_position(entity.get_position());
    // link body and entity
    link_foundation_entity->initialize(*body_foundation, entity);
    link_foundation_entity->set_constraints(true, true, true, true, true, true);
    is_linked = true;
}

void FoundationElastoBody::set_fixed(bool is_fixed) {
    body_foundation->set_fixed(is_fixed);
}

bool FoundationElastoBody::is_fixed() const {
    return body_foundation->is_fixed();
}

void FoundationElastoBody::build() {
    // fix foundation by default
    set_fixed(true);
}

void FoundationElastoBody::translate(const Vector3d& translation_vector) const {
    body_foundation->translate(translation_vector);
}

void FoundationElastoBody::rotate(double angle, const Vector3d& axis) const {
    body_foundation->rotate(angle, axis);
}

double FoundationElastoBody::get_mass() const {
    return body_foundation->get_mass();
}

void FoundationElastoBody::assemble_this(seahowl::elasto::SystemElasto& system) {
    system.add(*body_foundation);
    if (is_linked) {
        system.add(*link_foundation_entity);
    }
}
