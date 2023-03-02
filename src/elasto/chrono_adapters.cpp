#include <seahowl/elasto/chrono_adapters.h>
#include <chrono/core/ChVector.h>
#include <chrono/core/ChMatrix.h>
#include <chrono/fea/ChBeamSectionTaperedTimoshenkoFPM.h>
#include <chrono/fea/ChElementBeamTaperedTimoshenkoFPM.h>
#include <chrono/fea/ChElementBeamEuler.h>
#include <chrono/physics/ChLinkMate.h>
#include <chrono/physics/ChLinkRevolute.h>
#include <chrono/fea/ChMesh.h>
#include <chrono/physics/ChSystemSMC.h>

#include <seahowl/elasto/reference_point_elasto.h>
#include <seahowl/elasto/elasto.h>

#include <vector>
#include <memory>

namespace seahowl {
namespace elasto {

chrono::ChVector<double> vec2ch(Vector3d vector_in) {
    return chrono::ChVector<double>(vector_in[0], vector_in[1], vector_in[2]);
}
Vector3d ch2vec(chrono::ChVector<double> vector_in) {
    return Vector3d(vector_in[0], vector_in[1], vector_in[2]);
}

chrono::ChQuaternion<double> quat2ch(Quaternion quaternion_in) {
    return chrono::ChQuaternion<double>(quaternion_in.w(), quaternion_in.x(), quaternion_in.y(), quaternion_in.z());
}

Quaternion ch2quat(chrono::ChQuaternion<double> quaternion_in) {
    return Quaternion(quaternion_in[0], quaternion_in[1], quaternion_in[2], quaternion_in[3]);
}

RigidBodyChrono::RigidBodyChrono() {
    chobj = chrono_types::make_shared<chrono::ChBody>();
}

void RigidBodyChrono::set_position(Vector3d position) {
    chobj->SetPos(vec2ch(position));
}

void RigidBodyChrono::set_mass(double mass) {
    chobj->SetMass(mass);
}

void RigidBodyChrono::set_inertia_diagonal(Vector3d inertia) {
    chobj->SetInertiaXX(vec2ch(inertia));
}

void RigidBodyChrono::set_rotation(Quaternion rotation) {
    chobj->SetRot(quat2ch(rotation));
}

void RigidBodyChrono::reset_forces() {
    chobj->Empty_forces_accumulators();
}

void RigidBodyChrono::accumulate_torque(Vector3d torque, bool is_local) {
    chobj->Accumulate_torque(torque, is_local);
}

double RigidBodyChrono::get_mass() {
    return chobj->GetMass();
}

Vector3d RigidBodyChrono::get_position() const {
    return ch2vec(chobj->GetPos());
}

Vector3d RigidBodyChrono::get_velocity() const {
    return ch2vec(chobj->GetPos_dt());
}

Vector3d RigidBodyChrono::get_acceleration() const {
    return ch2vec(chobj->GetPos_dtdt());
}

Quaternion RigidBodyChrono::get_rotation() const {
    return ch2quat(chobj->GetRot());
}

Vector3d RigidBodyChrono::get_direction() const {
    return ch2vec(chobj->GetRot().GetVector());
}

Vector3d RigidBodyChrono::get_rotational_velocity_local() const {
    return ch2vec(chobj->GetWvel_loc());
}

Vector3d RigidBodyChrono::get_rotational_acceleration_local() const {
    return ch2vec(chobj->GetWacc_loc());
}

Vector3d RigidBodyChrono::get_rotational_velocity_global() const {
    return ch2vec(chobj->GetWvel_par());
}

Vector3d RigidBodyChrono::get_rotational_acceleration_global() const {
    return ch2vec(chobj->GetWacc_par());
}

NodeFEAChrono::NodeFEAChrono(Vector3d position, Quaternion rotation) {
    chobj =
        chrono_types::make_shared<chrono::fea::ChNodeFEAxyzrot>(chrono::ChFrame<>(vec2ch(position), quat2ch(rotation)));
}

void NodeFEAChrono::set_position(Vector3d position) {
    chobj->SetPos(vec2ch(position));
}

void NodeFEAChrono::set_rotation(Quaternion rotation) {
    chobj->SetRot(quat2ch(rotation));
}

void NodeFEAChrono::set_load(Vector3d force) {
    chobj->SetForce(vec2ch(force));
}

void NodeFEAChrono::set_torque(Vector3d torque) {
    chobj->SetTorque(vec2ch(torque));
}

Vector3d NodeFEAChrono::get_position() const {
    return ch2vec(chobj->GetPos());
}

Vector3d NodeFEAChrono::get_velocity() const {
    return ch2vec(chobj->GetPos_dt());
}

Vector3d NodeFEAChrono::get_acceleration() const {
    return ch2vec(chobj->GetPos_dtdt());
}

Quaternion NodeFEAChrono::get_rotation() const {
    return Quaternion(ch2quat(chobj->GetRot()));
}

Vector3d NodeFEAChrono::get_direction() const {
    return ch2vec(chobj->TransformDirectionLocalToParent(chrono::ChVector<double>(1.0, 0.0, 0.0)));
}

Vector3d NodeFEAChrono::get_rotational_velocity_local() const {
    return ch2vec(chobj->GetWvel_loc());
}

Vector3d NodeFEAChrono::get_rotational_acceleration_local() const {
    return ch2vec(chobj->GetWacc_loc());
}

Vector3d NodeFEAChrono::get_rotational_velocity_global() const {
    return ch2vec(chobj->GetWvel_par());
}

Vector3d NodeFEAChrono::get_rotational_acceleration_global() const {
    return ch2vec(chobj->GetWacc_par());
}

Vector3d NodeFEAChrono::get_load() const {
    return ch2vec(chobj->GetForce());
}

Vector3d NodeFEAChrono::get_torque() const {
    return ch2vec(chobj->GetTorque());
}

void NodeFEAChrono::set_properties(const BladeReferencePointElasto& ref) {
    section = chrono_types::make_shared<chrono::fea::ChBeamSectionTimoshenkoAdvancedGeneric>();
    // offsets
    section->SetCenterOfMass(ref.offset_gravity.y(), -ref.offset_gravity.x());
    section->SetCentroidY(ref.offset_elastic.y());
    section->SetCentroidZ(-ref.offset_elastic.x());
    // material properties
    section->SetMassPerUnitLength(ref.mass_matrix(0, 0));
    // axial
    section->SetAxialRigidity(ref.stiffness_matrix(0, 0));
    section->SetXtorsionRigidity(ref.stiffness_matrix(3, 3));
    // flap
    section->SetYbendingRigidity(ref.stiffness_matrix(4, 4));
    // edge
    section->SetZbendingRigidity(ref.stiffness_matrix(5, 5));
    // damping
    chrono::fea::DampingCoefficients damping_coefficients;
    damping_coefficients.bx = ref.damping_coefficients[0];
    damping_coefficients.by = ref.damping_coefficients[1];
    damping_coefficients.bz = ref.damping_coefficients[2];
    damping_coefficients.bt = ref.damping_coefficients[3];
    damping_coefficients.alpha = ref.damping_coefficients[4];
    section->SetBeamRaleyghDamping(damping_coefficients);
}

void NodeFEAChrono::set_properties(const TowerReferencePointElasto& ref) {
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

BladeElementFEAChrono::BladeElementFEAChrono() {
    chobj = chrono_types::make_shared<chrono::fea::ChElementBeamTaperedTimoshenko>();
    chobj_base = chobj;
    // create blade section
    auto blade_section = chrono_types::make_shared<chrono::fea::ChBeamSectionTaperedTimoshenkoAdvancedGeneric>();
    chobj->SetTaperedSection(blade_section);
}

void BladeElementFEAChrono::set_nodes(std::shared_ptr<NodeFEA> node1, std::shared_ptr<NodeFEA> node2) {
    nodes0.clear();
    nodes0.push_back(node1);
    nodes0.push_back(node2);

    // set nodes
    chobj->SetNodes(std::dynamic_pointer_cast<NodeFEAChrono>(node1)->chobj,
                    std::dynamic_pointer_cast<NodeFEAChrono>(node2)->chobj);
    // set tapered sections
    chobj->GetTaperedSection()->SetSectionA(std::dynamic_pointer_cast<NodeFEAChrono>(node1)->section);
    chobj->GetTaperedSection()->SetSectionB(std::dynamic_pointer_cast<NodeFEAChrono>(node2)->section);
}

void BladeElementFEAChrono::set_prebend(const Quaternion& prebend) {
    chobj->SetNodeBreferenceRot(quat2ch(prebend));
}

double BladeElementFEAChrono::get_mass() {
    return chobj->GetMass();
}

void BladeElementFEAChrono::evaluate_position_rotation(double eta, Vector3d& position, Quaternion& rotation) {
    auto chvec = vec2ch(position);
    auto chquat = quat2ch(rotation);
    chobj->EvaluateSectionFrame(eta, chvec, chquat);
    position[0] = chvec[0];
    position[1] = chvec[1];
    position[2] = chvec[2];
    rotation = ch2quat(chquat);
}

MooringElementFEAChrono::MooringElementFEAChrono() {
    chobj = chrono_types::make_shared<chrono::fea::ChElementBeamEuler>();
    chobj_base = chobj;
}

void MooringElementFEAChrono::set_nodes(std::shared_ptr<NodeFEA> node1, std::shared_ptr<NodeFEA> node2) {
    nodes0.clear();
    nodes0.push_back(node1);
    nodes0.push_back(node2);

    // set nodes
    chobj->SetNodes(std::dynamic_pointer_cast<NodeFEAChrono>(node1)->chobj,
                    std::dynamic_pointer_cast<NodeFEAChrono>(node2)->chobj);
}

double MooringElementFEAChrono::get_mass() {
    return chobj->GetMass();
}

void MooringElementFEAChrono::evaluate_position_rotation(double eta, Vector3d& position, Quaternion& rotation) {
    auto chvec = vec2ch(position);
    auto chquat = quat2ch(rotation);
    chobj->EvaluateSectionFrame(eta, chvec, chquat);
    position[0] = chvec[0];
    position[1] = chvec[1];
    position[2] = chvec[2];
    rotation = ch2quat(chquat);
}

void MooringElementFEAChrono::set_properties(double density, double diameter, double stiffness_axial) {
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

LinkFixChrono::LinkFixChrono() {
    chobj = chrono_types::make_shared<chrono::ChLinkMateFix>();
}

void LinkFixChrono::initialize(std::shared_ptr<RigidBody> body1, std::shared_ptr<RigidBody> body2) {
    chobj->Initialize(std::dynamic_pointer_cast<RigidBodyChrono>(body1)->chobj,
                      std::dynamic_pointer_cast<RigidBodyChrono>(body2)->chobj);
}

void LinkFixChrono::initialize(std::shared_ptr<NodeFEA> node1, std::shared_ptr<RigidBody> body2) {
    chobj->Initialize(std::dynamic_pointer_cast<NodeFEAChrono>(node1)->chobj,
                      std::dynamic_pointer_cast<RigidBodyChrono>(body2)->chobj);
}

Vector3d LinkFixChrono::get_reaction_force() const {
    return ch2vec(chobj->Get_react_force());
}

Vector3d LinkFixChrono::get_reaction_torque() const {
    return ch2vec(chobj->Get_react_torque());
}

LinkRevoluteChrono::LinkRevoluteChrono() {
    chobj = chrono_types::make_shared<chrono::ChLinkRevolute>();
}

void LinkRevoluteChrono::initialize(std::shared_ptr<RigidBody> body1, std::shared_ptr<RigidBody> body2) {
    chobj->Initialize(std::dynamic_pointer_cast<RigidBodyChrono>(body1)->chobj,
                      std::dynamic_pointer_cast<RigidBodyChrono>(body2)->chobj,
                      std::dynamic_pointer_cast<RigidBodyChrono>(body2)->chobj->GetFrame_COG_to_abs());
}

Vector3d LinkRevoluteChrono::get_reaction_force() const {
    return ch2vec(chobj->Get_react_force());
}

Vector3d LinkRevoluteChrono::get_reaction_torque() const {
    return ch2vec(chobj->Get_react_torque());
}

MeshElastoChrono::MeshElastoChrono() {
    chobj = chrono_types::make_shared<chrono::fea::ChMesh>();
}

void MeshElastoChrono::add(std::shared_ptr<NodeFEA> node) {
    chobj->AddNode(std::dynamic_pointer_cast<NodeFEAChrono>(node)->chobj);
}

void MeshElastoChrono::add(std::shared_ptr<ElementFEA> element) {
    chobj->AddElement(std::dynamic_pointer_cast<ElementFEAChrono>(element)->chobj_base);
}

void MeshElastoChrono::add(std::shared_ptr<chrono::fea::ChElementBeam> element) {
    chobj->AddElement(element);
}

SystemElastoChrono::SystemElastoChrono() {}

Vector3d SystemElastoChrono::get_gravitational_acceleration() const {
    return ch2vec(chobj.Get_G_acc());
}

void SystemElastoChrono::set_gravitational_acceleration(Vector3d gravitational_acceleration) {
    chobj.Set_G_acc(gravitational_acceleration);
}

void SystemElastoChrono::add(std::shared_ptr<RigidBody> body) {
    chobj.Add(std::dynamic_pointer_cast<RigidBodyChrono>(body)->chobj);
}

void SystemElastoChrono::add(std::shared_ptr<MeshElasto> mesh) {
    chobj.Add(std::dynamic_pointer_cast<MeshElastoChrono>(mesh)->chobj);
}

void SystemElastoChrono::add(std::shared_ptr<LinkFix> link) {
    chobj.Add(std::dynamic_pointer_cast<LinkFixChrono>(link)->chobj);
}

void SystemElastoChrono::add(std::shared_ptr<LinkRevolute> link) {
    chobj.Add(std::dynamic_pointer_cast<LinkRevoluteChrono>(link)->chobj);
}

}  // namespace elasto
}  // namespace seahowl
