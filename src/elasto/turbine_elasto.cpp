#include "seahowl/elasto/turbine_elasto.h"

#include "seahowl/elasto/system_elasto.h"
#include "seahowl/elasto/chrono_adapters.h"

using namespace seahowl::elasto;

TurbineElasto::TurbineElasto() {
    rna = RotorNacelleAssemblyElasto();
    tower = TowerElasto();
}

void TurbineElasto::assemble(SystemElasto& system) {
    // assemble rotor & tower
    rna.assemble(system);
    tower.assemble(system);
    // link tower to rotor
    link_rna_tower(system);
}

void TurbineElasto::link_rna_tower(SystemElasto& system) {
    auto towertop_node = tower.nodes[tower.nodes.size() - 1];
    // translate RNA center of origin to towertop
    rna.translate(towertop_node->get_position() - rna.body_yaw_bearing->get_position());
    rna.link_towertop_yaw_bearing = std::make_unique<LinkChrono>();
    system.add(*(rna.link_towertop_yaw_bearing.get()));
    rna.link_towertop_yaw_bearing->initialize(*(towertop_node.get()), *(rna.body_yaw_bearing.get()));
    rna.link_towertop_yaw_bearing->set_constraints(true, true, true, true, true, true);
}

void TurbineElasto::build() {
    // build rotor & tower
    rna.build();
    tower.build();
}

void TurbineElasto::translate(const Vector3d& translation_vector) const {
    rna.translate(translation_vector);
    tower.translate(translation_vector);
}

void TurbineElasto::rotate(double angle, const Vector3d& axis) const {
    rna.rotate(angle, axis);
    tower.rotate(angle, axis);
}

double TurbineElasto::get_mass() const {
    double total_mass = 0.0;
    // RNA
    total_mass += rna.get_mass();
    // tower
    total_mass += tower.get_mass();
    return total_mass;
}
