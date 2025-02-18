#include "seahowl/elasto/turbine_elasto.h"

#include "seahowl/elasto/system_elasto.h"
#include "seahowl/elasto/chrono_adapters.h"
#include "seahowl/elasto/foundation_elasto.h"

using namespace seahowl::elasto;

TurbineElasto::TurbineElasto() {
    rna = RotorNacelleAssemblyElasto();
    tower = TowerElasto();
    foundation = std::make_shared<FoundationElastoBody>();
}

void TurbineElasto::assemble_this(SystemElasto& system) {
    // assemble foundation
    if (foundation) {
        foundation->assemble(system);
    }
    // assemble RNA
    rna.assemble(system);
    // assemble tower
    tower.assemble(system);
}

void TurbineElasto::build() {
    // build RNA
    rna.build();
    // build tower
    tower.build();

    // link tower to rotor
    auto& towertop_node = *tower.nodes.back();
    // translate RNA center of origin to towertop
    rna.translate(towertop_node.get_position() - rna.body_mount->get_position());
    rna.attach_rna_to_node(towertop_node);

    // build foundation
    if (foundation) {
        foundation->build();
        foundation->link_to_entity(*tower.nodes.front());
    }
}

void TurbineElasto::presetup(double fraction) {
    if (foundation) {
        foundation->presetup(fraction);
    }
    rna.presetup(fraction);
    tower.presetup(fraction);
}

void TurbineElasto::translate(const Vector3d& translation_vector) const {
    rna.translate(translation_vector);
    tower.translate(translation_vector);
    if (foundation) {
        foundation->translate(translation_vector);
    }
}

void TurbineElasto::rotate(double angle, const Vector3d& axis) const {
    rna.rotate(angle, axis);
    tower.rotate(angle, axis);
    if (foundation) {
        foundation->rotate(angle, axis);
    }
}

double TurbineElasto::get_mass() const {
    double total_mass = 0.0;
    // RNA
    total_mass += rna.get_mass();
    // tower
    total_mass += tower.get_mass();
    // foundation
    if (foundation) {
        foundation->get_mass();
    }
    return total_mass;
}
