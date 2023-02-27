#include <seahowl/elasto/blade_elasto.h>

#include <seahowl/elasto/utils_elasto.h>  // WeightedElasto
#include <seahowl/core/utils.h>           // For DiscretizationPoint
#include <seahowl/elasto/reference_point_elasto.h>
#include <seahowl/elasto/utils_elasto.h>

#include <numeric>

#include <chrono/fea/ChBuilderBeam.h>
#include <chrono/fea/ChElementBeamTaperedTimoshenko.h>
#include <chrono/fea/ChElementBeamTaperedTimoshenkoFPM.h>
#include <chrono/fea/ChMesh.h>
#include <chrono/physics/ChSystemSMC.h>
#include <chrono/physics/ChLoadContainer.h>

using namespace seahowl::elasto;

BladeElasto::BladeElasto() {}

void BladeElasto::build() {
    // check that enough reference points were defined to create elements (at least 2)
    if (reference_points.size() <= 2) {
        throw std::runtime_error("Not enough elasto reference points defined for blade.");
    }

    // check that discretization_fractions was defined, otherwise take reference point fractions
    if (discretization_fractions.size() == 0) {
        for (int ii = 0; ii < reference_points.size(); ii++) {
            discretization_fractions.push_back(reference_points[ii].fraction);
        }
    } else if (discretization_fractions.size() == 1) {
        double npoints = discretization_fractions[0] + 1;
        double dp = 1.0 / (npoints - 1);
        discretization_fractions.clear();
        for (int ii = 0; ii < int(npoints); ii++) {
            discretization_fractions.push_back(ii * dp);
        }
    }

    // build
    discretized_points = seahowl::core::get_discretized_points(discretization_fractions, reference_points);
    ///@todo find better way to build ReferencePointElasto from BladeReferencePointElasto
    std::vector<ReferencePointElasto> discretized_points0;
    for (int ii = 0; ii < discretized_points.size(); ii++) {
        auto discretized_point0 = ReferencePointElasto();
        discretized_point0.coordinates = discretized_points[ii].coordinates;
        discretized_point0.fraction = discretized_points[ii].fraction;
        discretized_points0.push_back(discretized_point0);
    }
    build_nodes(discretized_points0);
    // apply structural twist
    for (int ii = 0; ii < nodes.size(); ii++) {
        auto& node = nodes[ii];
        auto& point = discretized_points[ii];
        auto axis = node->TransformDirectionLocalToParent(Vector3d(1.0, 0.0, 0.0));
        chrono::ChMatrix33<> twist_matrix(Q_from_AngAxis(-point.structural_twist, axis));
        nodes[ii]->Frame().SetRot(twist_matrix * chrono::ChMatrix33(nodes[ii]->Frame().coord.rot));
    }

    if (fpm_mode) {
        build_elements_tapered_timoshenko_fpm();
    } else {
        build_elements_tapered_timoshenko();
    }
    // commented out since loads are applied to nodes;
    // build_loads(system);
};

void BladeElasto::build_elements_tapered_timoshenko() {
    elements.clear();
    const auto nelements = nodes.size() - 1;

    if (nelements <= 0) {
        throw std::runtime_error("Trying to build blade with no element.");
    }

    // make first section for tapered section
    auto section = chrono_types::make_shared<chrono::fea::ChBeamSectionTimoshenkoAdvancedGeneric>();
    auto& discretized_point = discretized_points[0];
    // offsets
    section->SetCenterOfMass(discretized_point.offset_gravity.y(), -discretized_point.offset_gravity.x());
    section->SetCentroidY(discretized_point.offset_elastic.y());
    section->SetCentroidZ(-discretized_point.offset_elastic.x());
    // material properties
    section->SetMassPerUnitLength(discretized_point.mass_matrix(0, 0));
    // axial
    section->SetAxialRigidity(discretized_point.stiffness_matrix(0, 0));
    section->SetXtorsionRigidity(discretized_point.stiffness_matrix(3, 3));
    // flap
    section->SetYbendingRigidity(discretized_point.stiffness_matrix(4, 4));
    // edge
    section->SetZbendingRigidity(discretized_point.stiffness_matrix(5, 5));
    // damping
    section->SetBeamRaleyghDamping(discretized_point.damping_coefficients);

    for (size_t ii = 1; ii < nelements + 1; ii++) {
        // create element
        auto element = chrono_types::make_shared<chrono::fea::ChElementBeamTaperedTimoshenko>();
        // add element to blade elements vector
        elements.push_back(element);
        // set element nodes
        element->SetNodes(nodes[ii - 1], nodes[ii]);

        // create blade section
        auto blade_section = chrono_types::make_shared<chrono::fea::ChBeamSectionTaperedTimoshenkoAdvancedGeneric>();
        element->SetTaperedSection(blade_section);

        // set first section for tapered section
        blade_section->SetSectionA(section);

        // make second section for tapered section
        section = chrono_types::make_shared<chrono::fea::ChBeamSectionTimoshenkoAdvancedGeneric>();
        blade_section->SetSectionB(section);
        auto& discretized_point = discretized_points[ii];
        // offsets
        section->SetCenterOfMass(discretized_point.offset_gravity.y(), -discretized_point.offset_gravity.x());
        section->SetCentroidY(discretized_point.offset_elastic.y());
        section->SetCentroidZ(-discretized_point.offset_elastic.x());
        // material properties
        section->SetMassPerUnitLength(discretized_point.mass_matrix(0, 0));
        // axial
        section->SetAxialRigidity(discretized_point.stiffness_matrix(0, 0));
        section->SetXtorsionRigidity(discretized_point.stiffness_matrix(3, 3));
        // flap
        section->SetYbendingRigidity(discretized_point.stiffness_matrix(4, 4));
        // edge
        section->SetZbendingRigidity(discretized_point.stiffness_matrix(5, 5));
        // damping
        section->SetBeamRaleyghDamping(discretized_point.damping_coefficients);

        // apply prebend and structural twist
        auto rotation_relative = (nodes[ii]->GetRot() * nodes[ii - 1]->GetRot().GetInverse()).GetNormalized();
        // switch from IEC standard (Z along blade) to chrono element coordinate system (X along element)
        rotation_relative = chrono::ChQuaternion<>(rotation_relative[0], rotation_relative[3], rotation_relative[2],
                                                   rotation_relative[1]);
        element->SetNodeBreferenceRot(rotation_relative);
    }
}
void BladeElasto::build_elements_tapered_timoshenko_fpm() {
    elements.clear();
    const auto nelements = nodes.size() - 1;

    chrono::ChMatrixNM<double, 6, 6> mm;
    for (int jj = 0; jj < 6; jj++) {
        mm(jj, jj) = 1.0;
    }
    // make first section for tapered section
    auto section = chrono_types::make_shared<chrono::fea::ChBeamSectionTimoshenkoAdvancedGenericFPM>();
    auto& discretized_point = discretized_points[0];
    // offsets
    section->SetCenterOfMass(discretized_point.offset_gravity.y(), -discretized_point.offset_gravity.x());
    section->SetCentroidY(discretized_point.offset_elastic.y());
    section->SetCentroidZ(-discretized_point.offset_elastic.x());
    // material properties
    section->SetMassMatrixFPM(discretized_point.mass_matrix);
    section->SetStiffnessMatrixFPM(discretized_point.stiffness_matrix);
    // damping
    section->SetBeamRaleyghDamping(discretized_point.damping_coefficients);

    for (size_t ii = 1; ii < nelements + 1; ii++) {
        // create element
        auto element = chrono_types::make_shared<chrono::fea::ChElementBeamTaperedTimoshenkoFPM>();
        // add element to blade elements vector
        elements.push_back(element);
        // set element nodes
        element->SetNodes(nodes[ii - 1], nodes[ii]);

        // create blade section
        auto blade_section = chrono_types::make_shared<chrono::fea::ChBeamSectionTaperedTimoshenkoAdvancedGenericFPM>();
        element->SetTaperedSection(blade_section);

        // set first section for tapered section
        blade_section->SetSectionA(section);

        // make second section for tapered section
        section = chrono_types::make_shared<chrono::fea::ChBeamSectionTimoshenkoAdvancedGenericFPM>();
        blade_section->SetSectionB(section);
        auto& discretized_point = discretized_points[ii];
        // offsets
        section->SetCenterOfMass(discretized_point.offset_gravity.y(), -discretized_point.offset_gravity.x());
        section->SetCentroidY(discretized_point.offset_elastic.y());
        section->SetCentroidZ(-discretized_point.offset_elastic.x());
        // material properties
        section->SetMassMatrixFPM(discretized_point.mass_matrix);
        section->SetStiffnessMatrixFPM(discretized_point.stiffness_matrix);
        // damping
        section->SetBeamRaleyghDamping(discretized_point.damping_coefficients);

        // apply prebend and structural twist
        auto rotation_relative = (nodes[ii]->GetRot() * nodes[ii - 1]->GetRot().GetInverse()).GetNormalized();
        // switch from IEC standard (Z along blade) to chrono element coordinate system (X along element)
        rotation_relative = chrono::ChQuaternion<>(rotation_relative[0], rotation_relative[3], rotation_relative[2],
                                                   rotation_relative[1]);
        element->SetNodeBreferenceRot(rotation_relative);
    }
}

// void BladeElasto::build_loads(chrono::ChSystemSMC& system) {
//    auto loadcontainer = chrono_types::make_shared<chrono::ChLoadContainer>();
//    system.Add(loadcontainer);
//
//    for (auto element : elements) {
//        std::shared_ptr<chrono::ChLoad<ChLoaderWeighted>> loader_weighted(
//            new chrono::ChLoad<ChLoaderWeighted>(element));
//        loaders_aero.push_back(loader_weighted);
//        loadcontainer->Add(loader_weighted);
//    }
//}

void BladeElasto::set_damping_coefficients(double axial, double edge, double flap, double torsion) {
    const chrono::fea::DampingCoefficients damping_coefficients{axial, edge, flap, torsion};

    for (auto& point : reference_points) {
        point.damping_coefficients = damping_coefficients;
    }

    for (auto& element : elements) {
        auto section =
            std::dynamic_pointer_cast<chrono::fea::ChElementBeamTaperedTimoshenko>(element)->GetTaperedSection();
        section->GetSectionA()->SetBeamRaleyghDamping(damping_coefficients);
        section->GetSectionB()->SetBeamRaleyghDamping(damping_coefficients);
    }
}

void BladeElasto::evaluate_position_rotation(Vector3d& position,
                                             chrono::ChQuaternion<double>& rotation,
                                             int element_index,
                                             double eta) const {
    auto element = std::dynamic_pointer_cast<chrono::fea::ChElementBeamTaperedTimoshenko>(elements[element_index]);

    // // unfortunately line below does not always work (returns nans sometimes when fpm_mode is true)
    element->EvaluateSectionFrame(eta, position, rotation);
    auto w1 = std::abs(eta - 1.0) * 0.5;
    auto w2 = std::abs(eta + 1.0) * 0.5;
    position = w1 * element->GetNodeA()->GetPos() + w2 * element->GetNodeB()->GetPos();
    rotation = element->GetNodeA()->GetRot();
}

void BladeElasto::apply_pitch_increment(double pitch_increment) {
    // apply pitch from root node direction and position
    auto root_dir = nodes.front()->TransformDirectionLocalToParent(Vector3d(1.0, 0.0, 0.0));
    auto root_pos = nodes.front()->GetPos();
    translate(-root_pos);
    rotate(-pitch_increment, root_dir);
    translate(root_pos);
    pitch += pitch_increment;
};
