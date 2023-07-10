#include "seahowl/elasto/chrono_adapters.h"

#include "seahowl/elasto/reference_point_elasto.h"
#include "seahowl/elasto/entities_elasto.h"

#include <chrono/core/ChVector.h>
#include <chrono/core/ChMatrix.h>
#include <chrono/fea/ChBeamSectionTaperedTimoshenkoFPM.h>
#include <chrono/fea/ChElementBeamTaperedTimoshenkoFPM.h>
#include <chrono/fea/ChElementBeamEuler.h>
#include <chrono/physics/ChLinkMate.h>
#include <chrono/physics/ChLinkRevolute.h>
#include <chrono/fea/ChMesh.h>
#include <chrono/physics/ChSystemSMC.h>
#include <chrono/solver/ChDirectSolverLS.h>

#include <vector>
#include <memory>

namespace seahowl {
namespace elasto {

chrono::ChVector<double> vec2ch(const Vector3d& vector_in) {
    return chrono::ChVector<double>(vector_in[0], vector_in[1], vector_in[2]);
}
Vector3d ch2vec(const chrono::ChVector<double>& vector_in) {
    return Vector3d(vector_in[0], vector_in[1], vector_in[2]);
}

chrono::ChQuaternion<double> quat2ch(const Quaternion& quaternion_in) {
    return chrono::ChQuaternion<double>(quaternion_in.w(), quaternion_in.x(), quaternion_in.y(), quaternion_in.z());
}

Quaternion ch2quat(const chrono::ChQuaternion<double>& quaternion_in) {
    return Quaternion(quaternion_in[0], quaternion_in[1], quaternion_in[2], quaternion_in[3]);
}

Quaternion node_ch2iec(const chrono::ChQuaternion<double>& quaternion_in) {
    // Convert from Chrono standard ro IEC standard.
    // IEC convention:
    // x-axis: flapwise pointing towards nacelle,
    // y-axis : edgewise pointing towards trailing edge,
    // z-axis : longitudinal pointing towards blade tip.
    // Chrono convention:
    // x-axis: longitudinal pointing towards blade tip,
    // y-axis : edgewise pointing towards trailing edge,
    // z-axis : flapwise pointing away from nacelle.
    // ==> need to rotate +90 degrees around Chrono y-axis to transform to IEC convention.
    auto angle = +PI / 2.0;
    return ch2quat(quaternion_in) * Quaternion(cos(angle / 2), 0, sin(angle / 2), 0);
}

chrono::ChQuaternion<double> node_iec2ch(const Quaternion& quaternion_in) {
    // Convert from IEC standard to Chrono standard.
    // IEC convention:
    // x-axis: flapwise pointing towards nacelle,
    // y-axis : edgewise pointing towards trailing edge,
    // z-axis : longitudinal pointing towards blade tip.
    // Chrono convention:
    // x-axis: longitudinal pointing towards blade tip,
    // y-axis : edgewise pointing towards trailing edge,
    // z-axis : flapwise pointing away from nacelle.
    // ==> need to rotate -90 degrees around IEC y-axis to transform to Chrono convention.
    auto angle = -PI / 2.0;
    return quat2ch(quaternion_in * Quaternion(cos(angle / 2), 0, sin(angle / 2), 0));
}

Vector3d vec_ch2iec(const chrono::ChVector<double>& vector_in) {
    // Convert from Chrono standard ro IEC standard.
    // IEC convention:
    // x-axis: flapwise pointing towards nacelle,
    // y-axis : edgewise pointing towards trailing edge,
    // z-axis : longitudinal pointing towards blade tip.
    // Chrono convention:
    // x-axis: longitudinal pointing towards blade tip,
    // y-axis : edgewise pointing towards trailing edge,
    // z-axis : flapwise pointing away from nacelle.
    // ==> need to rotate +90 degrees around Chrono y-axis to transform to IEC convention.
    return Vector3d(-vector_in[2], vector_in[1], vector_in[0]);
}

void EntityDynamicChrono::set_position(const Vector3d& position) {
    chobj->SetPos(vec2ch(position));
}

Vector3d EntityDynamicChrono::get_position() const {
    return ch2vec(chobj->GetPos());
}

void EntityDynamicChrono::set_rotation(const Quaternion& rotation) {
    chobj->SetRot(quat2ch(rotation));
}

Quaternion EntityDynamicChrono::get_rotation() const {
    return ch2quat(chobj->GetRot());
}

void EntityDynamicChrono::set_velocity(const Vector3d& velocity) {
    chobj->SetPos_dt(vec2ch(velocity));
}

Vector3d EntityDynamicChrono::get_velocity() const {
    return ch2vec(chobj->GetPos_dt());
}

void EntityDynamicChrono::set_acceleration(const Vector3d& acceleration) {
    chobj->SetPos_dtdt(vec2ch(acceleration));
}

Vector3d EntityDynamicChrono::get_acceleration() const {
    return ch2vec(chobj->GetPos_dt());
}

void EntityDynamicChrono::set_rotational_velocity(const Vector3d& rotational_velocity) {
    chobj->SetWvel_par(vec2ch(rotational_velocity));
}

Vector3d EntityDynamicChrono::get_rotational_velocity() const {
    return ch2vec(chobj->GetWvel_par());
}

void EntityDynamicChrono::set_rotational_acceleration(const Vector3d& rotational_acceleration) {
    chobj->SetWacc_par(vec2ch(rotational_acceleration));
}

Vector3d EntityDynamicChrono::get_rotational_acceleration() const {
    return ch2vec(chobj->GetWacc_par());
}

BodyElastoChrono::BodyElastoChrono() {
    chobj = chrono_types::make_shared<chrono::ChBody>();
    EntityDynamicChrono::chobj = chobj;
}

void BodyElastoChrono::set_mass(double mass) {
    chobj->SetMass(mass);
}

void BodyElastoChrono::set_inertia_diagonal(const Vector3d& inertia) {
    chobj->SetInertiaXX(vec2ch(inertia));
}

void BodyElastoChrono::reset_forces() {
    chobj->Empty_forces_accumulators();
}

void BodyElastoChrono::accumulate_force(const Vector3d& force, bool is_local) {
    chobj->Accumulate_force(force, chobj->GetPos(), is_local);
}

void BodyElastoChrono::accumulate_torque(const Vector3d& torque, bool is_local) {
    chobj->Accumulate_torque(torque, is_local);
}

void BodyElastoChrono::set_fixed(bool is_fixed) {
    chobj->SetBodyFixed(is_fixed);
}

double BodyElastoChrono::get_mass() {
    return chobj->GetMass();
}

NodeElastoChrono::NodeElastoChrono(const Vector3d& position, const Quaternion& rotation) {
    chobj = chrono_types::make_shared<chrono::fea::ChNodeFEAxyzrot>(
        chrono::ChFrame<>(vec2ch(position), node_iec2ch(rotation)));
    EntityDynamicChrono::chobj = chobj;
}

void NodeElastoChrono::set_rotation(const Quaternion& rotation) {
    chobj->SetRot(node_iec2ch(rotation));
}

Quaternion NodeElastoChrono::get_rotation() const {
    return node_ch2iec(chobj->GetRot());
}

Vector3d NodeElastoChrono::get_direction() const {
    return ch2vec(chobj->TransformDirectionLocalToParent(chrono::ChVector<double>(1.0, 0.0, 0.0)));
}

void NodeElastoChrono::set_load(const Vector3d& force) {
    chobj->SetForce(vec2ch(force));
}

void NodeElastoChrono::set_torque(const Vector3d& torque) {
    chobj->SetTorque(vec2ch(torque));
}

void NodeElastoChrono::set_fixed(bool is_fixed) {
    chobj->SetFixed(is_fixed);
}

Vector3d NodeElastoChrono::get_load() const {
    return ch2vec(chobj->GetForce());
}

Vector3d NodeElastoChrono::get_torque() const {
    return ch2vec(chobj->GetTorque());
}

void NodeElastoChrono::set_properties(const BladeReferencePointElasto& ref, bool fpm) {
    Eigen::Matrix<double, 6, 6> mm = ref.mass_matrix.replicate(1, 1);
    Eigen::Matrix<double, 6, 6> sm = ref.stiffness_matrix.replicate(1, 1);
    // Convert from IEC convention to Chrono convention.
    // IEC standard:
    // x-axis: flapwise pointing towards nacelle,
    // y-axis : edgewise pointing towards trailing edge,
    // z-axis : longitudinal pointing towards blade tip.
    // Chrono convention:
    // x-axis: longitudinal pointing towards blade tip,
    // y-axis : edgewise pointing towards trailing edge,
    // z-axis : flapwise pointing away from nacelle.
    int jjo, kko;
    for (int jj = 0; jj < 6; jj++) {
        if (jj == 2 || jj == 5) {
            jjo = -2;
        }
        if (jj == 1 || jj == 4) {
            jjo = +0;
        }
        if (jj == 0 || jj == 3) {
            jjo = +2;
        }
        for (int kk = 0; kk < 6; kk++) {
            if (kk == 2 || kk == 5) {
                kko = -2;
            }
            if (kk == 1 || kk == 4) {
                kko = +0;
            }
            if (kk == 0 || kk == 3) {
                kko = +2;
            }
            mm(jj + jjo, kk + kko) = ref.mass_matrix(jj, kk);
            sm(jj + jjo, kk + kko) = ref.stiffness_matrix(jj, kk);
        }
    }
    if (fpm == true) {
        auto sectionFPM = chrono_types::make_shared<chrono::fea::ChBeamSectionTimoshenkoAdvancedGenericFPM>();
        section = sectionFPM;
        // offsets
        sectionFPM->SetCenterOfMass(ref.offset_gravity.y(), -ref.offset_gravity.x());
        sectionFPM->SetCentroidY(ref.offset_elastic.y());
        sectionFPM->SetCentroidZ(-ref.offset_elastic.x());
        // material properties
        sectionFPM->SetMassMatrixFPM(mm);
        sectionFPM->SetStiffnessMatrixFPM(sm);
        // damping
        chrono::fea::DampingCoefficients damping_coefficients;
        // damping coefficients: IEC -> Chrono convention
        damping_coefficients.bx = ref.damping_coefficients[2];
        damping_coefficients.by = ref.damping_coefficients[1];
        damping_coefficients.bz = ref.damping_coefficients[0];
        damping_coefficients.bt = ref.damping_coefficients[3];
        damping_coefficients.alpha = ref.damping_coefficients[4];
        sectionFPM->SetBeamRaleyghDamping(damping_coefficients);
    } else {
        section = chrono_types::make_shared<chrono::fea::ChBeamSectionTimoshenkoAdvancedGeneric>();
        // offsets
        section->SetCenterOfMass(ref.offset_gravity.y(), -ref.offset_gravity.x());
        section->SetCentroidY(ref.offset_elastic.y());
        section->SetCentroidZ(-ref.offset_elastic.x());
        // material properties
        section->SetMassPerUnitLength(mm(0, 0));
        // axial
        section->SetAxialRigidity(sm(0, 0));
        section->SetXtorsionRigidity(sm(3, 3));
        // flap
        section->SetYbendingRigidity(sm(4, 4));
        // edge
        section->SetZbendingRigidity(sm(5, 5));
        // damping
        chrono::fea::DampingCoefficients damping_coefficients;
        // damping coefficients: IEC -> Chrono convention
        damping_coefficients.bx = ref.damping_coefficients[2];
        damping_coefficients.by = ref.damping_coefficients[1];
        damping_coefficients.bz = ref.damping_coefficients[0];
        damping_coefficients.bt = ref.damping_coefficients[3];
        damping_coefficients.alpha = ref.damping_coefficients[4];
        section->SetBeamRaleyghDamping(damping_coefficients);
    }
}

void NodeElastoChrono::set_properties(const TowerReferencePointElasto& ref) {
    // make first section for tapered section
    section = chrono_types::make_shared<chrono::fea::ChBeamSectionTimoshenkoAdvancedGeneric>();
    // material properties
    section->SetMassPerUnitLength(ref.density);
    // axial
    section->SetAxialRigidity(ref.stiffness_axial);
    section->SetXtorsionRigidity(ref.stiffness_torsion);
    // foreaft
    section->SetZbendingRigidity(ref.stiffness_foreaft);
    // sideside
    section->SetYbendingRigidity(ref.stiffness_sideside);
    // damping
    chrono::fea::DampingCoefficients damping_coefficients;
    damping_coefficients.bx = ref.damping_coefficients[0];
    damping_coefficients.by = ref.damping_coefficients[1];
    damping_coefficients.bz = ref.damping_coefficients[2];
    damping_coefficients.bt = ref.damping_coefficients[3];
    damping_coefficients.alpha = ref.damping_coefficients[4];
    section->SetBeamRaleyghDamping(damping_coefficients);
}

void ElementElastoChrono::set_nodes(std::shared_ptr<NodeElasto> node1, std::shared_ptr<NodeElasto> node2) {
    nodes.clear();
    nodes.push_back(node1);
    nodes.push_back(node2);
}

void ElementElastoChrono::evaluate_position_rotation(double eta, Vector3d& position, Quaternion& rotation) {
    auto chvec = vec2ch(position);
    auto chquat = quat2ch(rotation);
    chobj->EvaluateSectionFrame(eta, chvec, chquat);
    position[0] = chvec[0];
    position[1] = chvec[1];
    position[2] = chvec[2];
    rotation = node_ch2iec(chquat);
}

void ElementElastoChrono::evaluate_force_torque(double eta, Vector3d& force, Vector3d& torque) {
    auto chforce = vec2ch(force);
    auto chtorque = vec2ch(torque);
    chobj->EvaluateSectionForceTorque(eta, chforce, chtorque);
    // convert Chrono convention to IEC
    force = vec_ch2iec(chforce);
    torque = vec_ch2iec(chtorque);
}

double ElementElastoChrono::get_mass() {
    return chobj->GetMass();
}

ElementBladeElastoChrono::ElementBladeElastoChrono() {
    chobj = chrono_types::make_shared<chrono::fea::ChElementBeamTaperedTimoshenko>();
    ElementElastoChrono::chobj = chobj;
    // create blade section
    auto blade_section = chrono_types::make_shared<chrono::fea::ChBeamSectionTaperedTimoshenkoAdvancedGeneric>();
    chobj->SetTaperedSection(blade_section);
}

void ElementBladeElastoChrono::set_nodes(std::shared_ptr<NodeElasto> node1, std::shared_ptr<NodeElasto> node2) {
    nodes.clear();
    nodes.push_back(node1);
    nodes.push_back(node2);

    // set nodes
    chobj->SetNodes(std::dynamic_pointer_cast<NodeElastoChrono>(node1)->chobj,
                    std::dynamic_pointer_cast<NodeElastoChrono>(node2)->chobj);
    // set tapered sections
    chobj->GetTaperedSection()->SetSectionA(std::dynamic_pointer_cast<NodeElastoChrono>(node1)->section);
    chobj->GetTaperedSection()->SetSectionB(std::dynamic_pointer_cast<NodeElastoChrono>(node2)->section);
}

void ElementBladeElastoChrono::set_prebend(const Quaternion& prebend) {
    // switch from IEC standard (Z along blade) to chrono element coordinate system (X along element)
    auto prebend_ch = chrono::ChQuaternion<double>(prebend.w(), prebend.z(), prebend.y(), prebend.x());
    chobj->SetNodeBreferenceRot(prebend_ch);
}

ElementBladeElastoChronoFPM::ElementBladeElastoChronoFPM() {
    chobj = chrono_types::make_shared<chrono::fea::ChElementBeamTaperedTimoshenkoFPM>();
    ElementElastoChrono::chobj = chobj;
    // create blade section
    auto blade_section = chrono_types::make_shared<chrono::fea::ChBeamSectionTaperedTimoshenkoAdvancedGenericFPM>();
    chobj->SetTaperedSection(blade_section);
}

void ElementBladeElastoChronoFPM::set_nodes(std::shared_ptr<NodeElasto> node1, std::shared_ptr<NodeElasto> node2) {
    nodes.clear();
    nodes.push_back(node1);
    nodes.push_back(node2);

    // set nodes
    chobj->SetNodes(std::dynamic_pointer_cast<NodeElastoChrono>(node1)->chobj,
                    std::dynamic_pointer_cast<NodeElastoChrono>(node2)->chobj);
    // set tapered sections
    chobj->GetTaperedSection()->SetSectionA(
        std::dynamic_pointer_cast<chrono::fea::ChBeamSectionTimoshenkoAdvancedGenericFPM>(
            std::dynamic_pointer_cast<NodeElastoChrono>(node1)->section));
    chobj->GetTaperedSection()->SetSectionB(
        std::dynamic_pointer_cast<chrono::fea::ChBeamSectionTimoshenkoAdvancedGenericFPM>(
            std::dynamic_pointer_cast<NodeElastoChrono>(node2)->section));
}

void ElementBladeElastoChronoFPM::set_prebend(const Quaternion& prebend) {
    // switch from IEC standard (Z along blade) to chrono element coordinate system (X along element)
    auto prebend_ch = chrono::ChQuaternion<double>(prebend.w(), prebend.z(), prebend.y(), prebend.x());
    chobj->SetNodeBreferenceRot(prebend_ch);
}

ElementMooringElastoChrono::ElementMooringElastoChrono() {
    chobj = chrono_types::make_shared<chrono::fea::ChElementBeamEuler>();
    ElementElastoChrono::chobj = chobj;
}

void ElementMooringElastoChrono::set_nodes(std::shared_ptr<NodeElasto> node1, std::shared_ptr<NodeElasto> node2) {
    nodes.clear();
    nodes.push_back(node1);
    nodes.push_back(node2);

    // set nodes
    chobj->SetNodes(std::dynamic_pointer_cast<NodeElastoChrono>(node1)->chobj,
                    std::dynamic_pointer_cast<NodeElastoChrono>(node2)->chobj);
}

void ElementMooringElastoChrono::set_properties(double density, double diameter, double stiffness_axial) {
    // create mooring section
    auto section = chrono_types::make_shared<chrono::fea::ChBeamSectionEulerAdvanced>();
    chobj->SetSection(section);
    section->SetDensity(density);
    double area = chrono::CH_C_PI * pow(diameter, 2) / 4.0;
    section->SetArea(area);
    section->SetYoungModulus(stiffness_axial / area);
    section->SetGshearModulus(0.0);
    section->SetAsCircularSection(diameter);
}

LinkChrono::LinkChrono() {
    chobj = chrono_types::make_shared<chrono::ChLinkMateGeneric>();
    chobj->SetConstrainedCoords(true, true, true, true, true, true);
}

void LinkChrono::initialize(BodyElasto& body1, BodyElasto& body2) {
    chobj->Initialize(dynamic_cast<BodyElastoChrono&>(body1).chobj, dynamic_cast<BodyElastoChrono&>(body2).chobj,
                      dynamic_cast<BodyElastoChrono&>(body2).chobj->GetFrame_COG_to_abs());
}

void LinkChrono::initialize(NodeElasto& node1, BodyElasto& body2) {
    chobj->Initialize(dynamic_cast<NodeElastoChrono&>(node1).chobj, dynamic_cast<BodyElastoChrono&>(body2).chobj,
                      dynamic_cast<BodyElastoChrono&>(body2).chobj->GetFrame_COG_to_abs());
}

void LinkChrono::initialize(NodeElasto& node1, NodeElasto& node2) {
    chobj->Initialize(dynamic_cast<NodeElastoChrono&>(node1).chobj, dynamic_cast<NodeElastoChrono&>(node2).chobj,
                      dynamic_cast<NodeElastoChrono&>(node2).chobj->Frame());
}

void LinkChrono::set_constraints(bool surge, bool sway, bool heave, bool roll, bool pitch, bool yaw) {
    chobj->SetConstrainedCoords(surge, sway, heave, roll, pitch, yaw);
}

Vector3d LinkChrono::get_reaction_force() const {
    return ch2vec(chobj->Get_react_force());
}

Vector3d LinkChrono::get_reaction_torque() const {
    return ch2vec(chobj->Get_react_torque());
}

MeshElastoChrono::MeshElastoChrono() {
    chobj = chrono_types::make_shared<chrono::fea::ChMesh>();
}

void MeshElastoChrono::add(NodeElasto& node) {
    chobj->AddNode(dynamic_cast<NodeElastoChrono&>(node).chobj);
}

void MeshElastoChrono::add(ElementElasto& element) {
    chobj->AddElement(dynamic_cast<ElementElastoChrono&>(element).chobj);
}

SystemElastoChrono::SystemElastoChrono() {
    chobj = chrono_types::make_shared<chrono::ChSystemSMC>();

    // solver
    auto solver = chrono_types::make_shared<chrono::ChSolverSparseLU>();
    chobj->SetSolver(solver);
    solver->UseSparsityPatternLearner(true);
    solver->LockSparsityPattern(true);
    solver->SetVerbose(false);

    // timestepping
    chobj->SetTimestepperType(chrono::ChTimestepper::Type::HHT);
    auto mystepper = std::dynamic_pointer_cast<chrono::ChTimestepperHHT>(chobj->GetTimestepper());
    mystepper->SetStepControl(false);
    mystepper->SetModifiedNewton(false);

    // make mesh
    mesh = std::make_shared<MeshElastoChrono>();
    add(*(mesh.get()));
}

void SystemElastoChrono::step(double dt) {
    chobj->DoStepDynamics(dt);
}

double SystemElastoChrono::get_time() const {
    return chobj->GetChTime();
}

void SystemElastoChrono::do_statics(bool linear, int nonlinear_steps) {
    // constrain rotor
    for (auto& turbine : turbines) {
        turbine.rotor.link_shaft_hub->set_constraints(true, true, true, true, true, true);
    }
    // linear statics
    if (linear) {
        chobj->DoStaticLinear();
    }
    // nonlinear statics
    if (nonlinear_steps > 0) {
        chobj->DoStaticNonlinear(nonlinear_steps, true);
    }
    // unconstrain rotor
    for (auto& turbine : turbines) {
        turbine.rotor.link_shaft_hub->set_constraints(true, true, true, false, true, true);
    }
};

Vector3d SystemElastoChrono::get_gravitational_acceleration() const {
    return ch2vec(chobj->Get_G_acc());
}

void SystemElastoChrono::set_gravitational_acceleration(const Vector3d& gravitational_acceleration) {
    chobj->Set_G_acc(gravitational_acceleration);
}

void SystemElastoChrono::add(BodyElasto& body) {
    chobj->Add(dynamic_cast<BodyElastoChrono&>(body).chobj);
}

void SystemElastoChrono::add(MeshElasto& mesh) {
    chobj->Add(dynamic_cast<MeshElastoChrono&>(mesh).chobj);
}

void SystemElastoChrono::add(Link& link) {
    chobj->Add(dynamic_cast<LinkChrono&>(link).chobj);
}

}  // namespace elasto
}  // namespace seahowl
