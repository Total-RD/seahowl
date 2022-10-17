#include <seahowl/elasto/tower_elasto.h>
#include <seahowl/core/utils.h>

#include <memory>
#include <vector>
#include <numeric>

#include <chrono/fea/ChMesh.h>
#include <chrono/fea/ChElementBeamTaperedTimoshenko.h>

using seahowl::elasto::TowerElasto;

TowerElasto::TowerElasto() {}

TowerElasto::~TowerElasto() {}

void TowerElasto::build() {
    // check that enough reference points were defined to create elements (at least 2)
    if (reference_points.size() <= 2) {
        throw std::runtime_error("Not enough elasto reference points defined for blade.");
    }

    // check that discretization_fractions was defined, otherwise take reference point fractions
    if (discretization_fractions.size() == 0) {
        for (int ii = 0; ii < reference_points.size(); ii++) {
            discretization_fractions.push_back(reference_points[ii].fraction);
        }
    }

    // build
    discretized_points = seahowl::core::get_discretized_points(discretization_fractions, reference_points);
    ///@todo find better way to build ReferencePointElasto from TowerReferencePointElasto
    std::vector<ReferencePointElasto> discretized_points0;
    for (int ii = 0; ii < discretized_points.size(); ii++) {
        auto discretized_point0 = ReferencePointElasto();
        discretized_point0.coordinates = discretized_points[ii].coordinates;
        discretized_point0.fraction = discretized_points[ii].fraction;
        discretized_points0.push_back(discretized_point0);
    }
    build_nodes(discretized_points0);

    build_elements_tapered_timoshenko();
};

void TowerElasto::build_elements_tapered_timoshenko() {
    elements.clear();
    const auto nelements = nodes.size() - 1;

    // make first section for tapered section
    auto section = chrono_types::make_shared<chrono::fea::ChBeamSectionTimoshenkoAdvancedGeneric>();
    auto& discretized_point = discretized_points[0];
    // material properties
    section->SetMassPerUnitLength(discretized_point.density);
    // axial
    section->SetAxialRigidity(discretized_point.stiffness_axial);
    section->SetXtorsionRigidity(discretized_point.stiffness_torsion);
    // foreaft
    section->SetZbendingRigidity(discretized_point.stiffness_foreaft);
    // sideside
    section->SetYbendingRigidity(discretized_point.stiffness_sideside);
    // damping
    section->SetBeamRaleyghDamping(discretized_point.damping_coefficients);

    for (size_t ii = 1; ii < nelements + 1; ii++) {
        // create element
        auto element = chrono_types::make_shared<chrono::fea::ChElementBeamTaperedTimoshenko>();
        // add element to tower elements vector
        elements.push_back(element);
        // set element nodes
        element->SetNodes(nodes[ii - 1], nodes[ii]);

        // create tower section
        auto tower_section = chrono_types::make_shared<chrono::fea::ChBeamSectionTaperedTimoshenkoAdvancedGeneric>();
        element->SetTaperedSection(tower_section);

        // set first section for tapered section
        tower_section->SetSectionA(section);

        // make second section for tapered section
        section = chrono_types::make_shared<chrono::fea::ChBeamSectionTimoshenkoAdvancedGeneric>();
        tower_section->SetSectionB(section);
        auto discretized_point = discretized_points[ii];
        // material properties
        section->SetMassPerUnitLength(discretized_point.density);
        // axial
        section->SetAxialRigidity(discretized_point.stiffness_axial);
        section->SetXtorsionRigidity(discretized_point.stiffness_torsion);
        // foreaft
        section->SetZbendingRigidity(discretized_point.stiffness_foreaft);
        // sideside
        section->SetYbendingRigidity(discretized_point.stiffness_sideside);
        // damping
        section->SetBeamRaleyghDamping(discretized_point.damping_coefficients);
    }
}

void TowerElasto::set_damping_coefficients(double axial, double edge, double flap, double torsion) {
    chrono::fea::DampingCoefficients damping_coefficients;
    damping_coefficients.bx = axial;
    damping_coefficients.by = edge;
    damping_coefficients.bz = flap;
    damping_coefficients.bt = torsion;
    for (int ii = 0; ii < reference_points.size(); ii++) {
        auto reference_point = reference_points[ii];
        reference_point.damping_coefficients = damping_coefficients;
    }
    for (int ii = 0; ii < elements.size(); ii++) {
        auto section =
            std::dynamic_pointer_cast<chrono::fea::ChElementBeamTaperedTimoshenko>(elements[ii])->GetTaperedSection();
        section->GetSectionA()->SetBeamRaleyghDamping(damping_coefficients);
        section->GetSectionB()->SetBeamRaleyghDamping(damping_coefficients);
    }
}