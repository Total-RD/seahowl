#include <seahowl/core/tower.h>

using namespace seahowl::core;
using namespace seahowl::elasto;
using namespace seahowl::aero;

Tower::Tower() {
    elasto = TowerElasto();
    aero = TowerAero();
}

void Tower::init(double time, double dt) {
    prestep(time, dt);
    poststep(time, dt);
}

void Tower::prestep(double time, double dt) {
    // update loads on elasto part
    update_loads_elasto();
}

void Tower::poststep(double time, double dt) {
    // update position of aero points
    update_positions_aero();
}

void Tower::assemble(std::shared_ptr<MeshElasto> mesh) {
    elasto.assemble(mesh);
}

void Tower::build() {
    // push reference points
    elasto.reference_points.clear();
    aero.reference_points.clear();
    for (auto& point : reference_points) {
        elasto.reference_points.push_back(TowerReferencePointElasto(point));
        aero.reference_points.push_back(TowerReferencePointAero(point));
    }
    // build
    elasto.build();
    aero.build();

    // mappings
    compute_mapping_aero2elasto();
    compute_mapping_elasto2aero();
}

void Tower::set_discretization_elasto(std::vector<double> fractions) {
    elasto.discretization_fractions = fractions;
}

void Tower::set_discretization_aero(std::vector<double> fractions) {
    aero.discretization_fractions = fractions;
}

void Tower::compute_mapping_aero2elasto() {
    // get aero element position (center) from which loads will be applied
    std::vector<double> aero_discretization_fractions;
    for (int ii = 0; ii < aero.elements.size(); ii++) {
        aero_discretization_fractions.push_back(aero.elements[ii].properties.fraction);
    }
    mapping_aero2elasto = get_indice_and_positions(aero_discretization_fractions, elasto.discretization_fractions);
}

void Tower::compute_mapping_elasto2aero() {
    mapping_elasto2aero = get_indice_and_positions(elasto.discretization_fractions, aero.discretization_fractions);
}

void Tower::update_positions_aero() {
    for (int ii = 0; ii < aero.elements.size(); ii++) {
        // update position and rotation of aero elements
        int elasto_element_index = mapping_aero2elasto[ii].index;
        auto element_elasto = elasto.elements[elasto_element_index];
        double eta = mapping_aero2elasto[ii].eta;
        auto& element_aero = aero.elements[ii];
        elasto.evaluate_position_rotation(element_aero.properties.coordinates, element_aero.properties.rotation,
                                          elasto_element_index, eta);

        // update velocity of aero elements
        aero.elements[ii].properties.velocity =
            0.5 * (std::dynamic_pointer_cast<chrono::fea::ChNodeFEAxyzrot>(element_elasto->GetNodeN(0))->GetPos_dt() +
                   std::dynamic_pointer_cast<chrono::fea::ChNodeFEAxyzrot>(element_elasto->GetNodeN(1))->GetPos_dt());
    }
}

void Tower::update_loads_elasto() {
    elasto.reset_loads();
    if (aero.loads.size() != mapping_aero2elasto.size()) {
        throw std::runtime_error("length of vector of loads and aero to elasto mapping do not match.");
    }
    auto offset = Vector3d(0.0, 0.0, 0.0);
    for (int ii = 0; ii < aero.loads.size(); ii++) {
        elasto.accumulate_element_load(aero.loads[ii], mapping_aero2elasto[ii].index, mapping_aero2elasto[ii].eta,
                                       offset);
    }
}
