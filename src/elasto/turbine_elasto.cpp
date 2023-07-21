#include "seahowl/elasto/turbine_elasto.h"

#include "seahowl/elasto/system_elasto.h"
#include "seahowl/elasto/chrono_adapters.h"

using namespace seahowl::elasto;

TurbineElasto::TurbineElasto() {
    rotor = RotorNacelleAssemblyElasto();
    tower = TowerElasto();
}

void TurbineElasto::assemble(SystemElasto& system) {
    // assemble rotor & tower
    rotor.assemble(system);
    tower.assemble(system);
    // link tower to rotor
    link_rna_tower(system);
}

void TurbineElasto::link_rna_tower(SystemElasto& system) {
    auto towertop_node = tower.nodes[tower.nodes.size() - 1];
    // translate RNA center of origin to towertop
    rotor.translate(towertop_node->get_position() - rotor.body_yaw_bearing->get_position());
    rotor.link_towertop_yaw_bearing = std::make_unique<LinkChrono>();
    system.add(*(rotor.link_towertop_yaw_bearing.get()));
    rotor.link_towertop_yaw_bearing->initialize(*(towertop_node.get()), *(rotor.body_yaw_bearing.get()));
    rotor.link_towertop_yaw_bearing->set_constraints(true, true, true, true, true, true);
}

void TurbineElasto::build() {
    // build rotor & tower
    rotor.build();
    tower.build();
}

void TurbineElasto::translate(Vector3d translation_vector) {
    rotor.translate(translation_vector);
    tower.translate(translation_vector);
}

void TurbineElasto::rotate(double angle, Vector3d axis) {
    rotor.rotate(angle, axis);
    tower.rotate(angle, axis);
}
