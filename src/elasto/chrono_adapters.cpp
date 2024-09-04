#include "seahowl/elasto/chrono_adapters.h"

#include "seahowl/elasto/reference_point_elasto.h"
#include "seahowl/elasto/entities_elasto.h"

#include <chrono/core/ChVector.h>
#include <chrono/core/ChMatrix.h>
#include <chrono/fea/ChBeamSectionTaperedTimoshenkoFPM.h>
#include <chrono/fea/ChElementBeamTaperedTimoshenkoFPM.h>
#include <chrono/fea/ChElementCableANCF.h>
#include <chrono/physics/ChLinkMate.h>
#include <chrono/physics/ChLoadsBody.h>
#include <chrono/physics/ChLoadContainer.h>
#include <chrono/fea/ChLinkPointPoint.h>
#include <chrono/fea/ChLinkPointFrame.h>
#include <chrono/physics/ChLinkRevolute.h>
#include <chrono/fea/ChMesh.h>
#include <chrono/physics/ChSystemSMC.h>
#include <chrono/solver/ChDirectSolverLS.h>

#include <vector>
#include <memory>
#include <spdlog/spdlog.h>

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

Vector3d vec_iec2ch(const Vector3d& vector_in) {
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
    return Vector3d(vector_in[2], vector_in[1], -vector_in[0]);
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
    return ch2vec(chobj->GetPos_dtdt());
}

void EntityDynamicChrono::set_rotational_velocity(const Vector3d& rotational_velocity, bool is_local) {
    if (is_local) {
        chobj->SetWvel_loc(vec2ch(rotational_velocity));
    } else {
        chobj->SetWvel_par(vec2ch(rotational_velocity));
    }
}

Vector3d EntityDynamicChrono::get_rotational_velocity(bool is_local) const {
    if (is_local) {
        return ch2vec(chobj->GetWvel_loc());
    } else {
        return ch2vec(chobj->GetWvel_par());
    }
}

void EntityDynamicChrono::set_rotational_acceleration(const Vector3d& rotational_acceleration, bool is_local) {
    if (is_local) {
        chobj->SetWacc_loc(vec2ch(rotational_acceleration));
    } else {
        chobj->SetWacc_par(vec2ch(rotational_acceleration));
    }
}

Vector3d EntityDynamicChrono::get_rotational_acceleration(bool is_local) const {
    if (is_local) {
        return ch2vec(chobj->GetWacc_loc());
    } else {
        return ch2vec(chobj->GetWacc_par());
    }
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

void BodyElastoChrono::set_inertia_matrix(const Eigen::Matrix<double, 3, 3>& inertia) {
    chobj->SetInertia(inertia);
};

Eigen::Matrix<double, 3, 3> BodyElastoChrono::get_inertia_matrix() const {
    return chobj->GetInertia();
}

void BodyElastoChrono::reset_loads() {
    chobj->Empty_forces_accumulators();
}

Vector3d BodyElastoChrono::get_force(bool is_local) const {
    if (is_local) {
        return get_rotation().inverse() * ch2vec(chobj->Get_accumulated_force());
    } else {
        return ch2vec(chobj->Get_accumulated_force());
    }
}

Vector3d BodyElastoChrono::get_torque(bool is_local) const {
    if (is_local) {
        return ch2vec(chobj->Get_accumulated_torque());
    } else {
        return get_rotation() * ch2vec(chobj->Get_accumulated_torque());
    }
}

void BodyElastoChrono::set_force(const Vector3d& force, bool is_local) {
    auto torque = get_torque(true);      // get previously accumulated torque
    chobj->Empty_forces_accumulators();  // empty accumulated forces and torques
    accumulate_torque(torque, true);     // set previously accumulated torque
    accumulate_force(force, is_local);
}

void BodyElastoChrono::set_torque(const Vector3d& torque, bool is_local) {
    auto force = get_force(false);       // get previously accumulated force
    chobj->Empty_forces_accumulators();  // empty accumulated forces and torques
    accumulate_force(force, false);      // set previously accumulated force
    accumulate_torque(torque, is_local);
}

void BodyElastoChrono::accumulate_force(const Vector3d& force, bool is_local) {
    if (is_local) {
        chobj->Accumulate_force(force, Vector3d(0.0, 0.0, 0.0), is_local);
    } else {
        chobj->Accumulate_force(force, chobj->GetPos(), is_local);
    }
}

void BodyElastoChrono::accumulate_torque(const Vector3d& torque, bool is_local) {
    chobj->Accumulate_torque(torque, is_local);
}

void BodyElastoChrono::set_fixed(bool is_fixed) {
    chobj->SetBodyFixed(is_fixed);
}

bool BodyElastoChrono::is_fixed() const {
    return chobj->GetBodyFixed();
}

double BodyElastoChrono::get_mass() {
    return chobj->GetMass();
}

NodeElastoChrono::NodeElastoChrono(const Vector3d& position, const Quaternion& rotation) {
    chobj = chrono_types::make_shared<chrono::fea::ChNodeFEAxyzrot>(
        chrono::ChFrame<>(vec2ch(position), node_iec2ch(rotation)));
    EntityDynamicChrono::chobj = chobj;
    NodeElastoChronoBase::chobj = chobj;
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

void NodeElastoChrono::reset_loads() {
    set_force(Vector3d(0.0, 0.0, 0.0), false);
    set_torque(Vector3d(0.0, 0.0, 0.0), true);
}

Vector3d NodeElastoChrono::get_force(bool is_local) const {
    if (is_local) {
        return get_rotation().inverse() * ch2vec(chobj->GetForce());
    } else {
        return ch2vec(chobj->GetForce());
    }
}

Vector3d NodeElastoChrono::get_torque(bool is_local) const {
    if (is_local) {
        return vec_ch2iec(chobj->GetTorque());
    } else {
        return get_rotation() * vec_ch2iec(chobj->GetTorque());
    }
}

void NodeElastoChrono::set_force(const Vector3d& force, bool is_local) {
    if (is_local) {
        chobj->SetForce(vec2ch(get_rotation() * force));
    } else {
        chobj->SetForce(vec2ch(force));
    }
}

void NodeElastoChrono::set_torque(const Vector3d& torque, bool is_local) {
    if (is_local) {
        chobj->SetTorque(vec_iec2ch(torque));
    } else {
        // chobj->SetTorque(chobj->TransformDirectionParentToLocal(vec2ch(torque)));
        chobj->SetTorque(vec_iec2ch(get_rotation().inverse() * torque));
    }
}

void NodeElastoChrono::accumulate_force(const Vector3d& force, bool is_local) {
    set_force(get_force(is_local) + force, is_local);
}

void NodeElastoChrono::accumulate_torque(const Vector3d& torque, bool is_local) {
    set_torque(get_torque(is_local) + torque, is_local);
}

void NodeElastoChrono::set_fixed(bool is_fixed) {
    chobj->SetFixed(is_fixed);
}

bool NodeElastoChrono::is_fixed() const {
    return chobj->IsFixed();
}

void NodeElastoChrono::set_properties(const BladeReferencePointElasto& ref, bool fpm) {
    // Convert from IEC convention to Chrono convention.
    // IEC standard:
    // x-axis: flapwise pointing towards nacelle,
    // y-axis : edgewise pointing towards trailing edge,
    // z-axis : longitudinal pointing towards blade tip.
    // Chrono convention:
    // x-axis: longitudinal pointing towards blade tip,
    // y-axis : edgewise pointing towards trailing edge,
    // z-axis : flapwise pointing away from nacelle.
    Eigen::Matrix<double, 3, 3> rot33 = AngleAxisd(PI / 2, Vector3d(0.0, 1.0, 0.0)).toRotationMatrix();
    Eigen::Matrix<double, 6, 6> rot66 = Eigen::Matrix<double, 6, 6>::Zero();
    for (int ii = 0; ii < 3; ii++) {
        for (int jj = 0; jj < 3; jj++) {
            rot66(ii, jj) = rot33(ii, jj);
            rot66(ii + 3, jj + 3) = rot33(ii, jj);
        }
    }
    auto mm = rot66 * ref.mass_matrix * rot66.transpose();
    auto sm = rot66 * ref.stiffness_matrix * rot66.transpose();

    if (ref.damping_coefficients.size() != 5) {
        throw std::runtime_error("Damping coefficients for blade must be a vector of length 5 (got " +
                                 std::to_string(ref.damping_coefficients.size()) + ").");
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
        // see ChBeamSectionTimoshenkoAdvancedGenericFPM methods: SetMassMatrixFPM, SetStiffnessMatrixFPM
        section->SetMassPerUnitLength(mm(0, 0));
        // inertias per unit length in this order: mIyy, mIzz, mIyz, mQy, mQz
        section->SetInertiasPerUnitLength(mm(4, 4), mm(5, 5), -mm(4, 5), mm(0, 4), -mm(0, 5));
        // axial
        section->SetXtorsionRigidity(sm(3, 3));
        section->SetAxialRigidity(sm(0, 0));
        // flap
        section->SetYbendingRigidity(sm(4, 4));
        section->SetYshearRigidity(sm(1, 1));
        // edge
        section->SetZbendingRigidity(sm(5, 5));
        section->SetZshearRigidity(sm(2, 2));
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
    section->SetInertiasPerUnitLength(ref.inertia_foreaft, ref.inertia_sideside, 0.0, 0.0, 0.0);
    // axial
    section->SetAxialRigidity(ref.stiffness_axial);
    section->SetXtorsionRigidity(ref.stiffness_torsion);
    // foreaft
    section->SetYbendingRigidity(ref.stiffness_foreaft);
    // sideside
    section->SetZbendingRigidity(ref.stiffness_sideside);
    // damping
    if (ref.damping_coefficients.size() != 5) {
        throw std::runtime_error("Damping coefficients for tower must be a vector of length 5 (got " +
                                 std::to_string(ref.damping_coefficients.size()) + ").");
    }
    chrono::fea::DampingCoefficients damping_coefficients;
    damping_coefficients.bx = ref.damping_coefficients[0];
    damping_coefficients.by = ref.damping_coefficients[1];
    damping_coefficients.bz = ref.damping_coefficients[2];
    damping_coefficients.bt = ref.damping_coefficients[3];
    damping_coefficients.alpha = ref.damping_coefficients[4];
    section->SetBeamRaleyghDamping(damping_coefficients);
}

NodeElastoChronoD::NodeElastoChronoD(const Vector3d& position, const Vector3d& direction) {
    chobj = chrono_types::make_shared<chrono::fea::ChNodeFEAxyzD>(vec2ch(position), vec2ch(direction));
    NodeElastoChronoBase::chobj = chobj;
}

void NodeElastoChronoD::set_rotation(const Quaternion& rotation) {
    // set rotation assuming direction of node to be local Z
    chobj->SetD(vec2ch(rotation * Vector3d(0.0, 0.0, 1.0)));
}

Quaternion NodeElastoChronoD::get_rotation() const {
    // get rotation assuming direction of node to be local Z
    // note: ChNodeFEAxyzD does not have a true rotation, only a direction
    return Quaternion::FromTwoVectors(Vector3d(0.0, 0.0, 1.0), get_direction());
}

Vector3d NodeElastoChronoD::get_direction() const {
    return ch2vec(chobj->GetD());
}

void NodeElastoChronoD::reset_loads() {
    set_force(Vector3d(0.0, 0.0, 0.0), false);
    set_torque(Vector3d(0.0, 0.0, 0.0), true);
}

Vector3d NodeElastoChronoD::get_force(bool is_local) const {
    if (is_local) {
        throw std::runtime_error("Cannot get force locally from ChNodeFEAxyzD.");
    } else {
        return ch2vec(chobj->GetForce());
    }
}

Vector3d NodeElastoChronoD::get_torque(bool is_local) const {
    // no torque on ChNodeFEAxyzD
    return Vector3d(0.0, 0.0, 0.0);
}

void NodeElastoChronoD::set_force(const Vector3d& force, bool is_local) {
    if (is_local) {
        throw std::runtime_error("Cannot set force locally for ChNodeFEAxyzD.");
    } else {
        chobj->SetForce(vec2ch(force));
    }
}

void NodeElastoChronoD::set_torque(const Vector3d& torque, bool is_local) {
    // no torque on ChNodeFEAxyzD
}

void NodeElastoChronoD::accumulate_force(const Vector3d& force, bool is_local) {
    set_force(get_force(is_local) + force, is_local);
}

void NodeElastoChronoD::accumulate_torque(const Vector3d& torque, bool is_local) {
    set_torque(get_torque(is_local) + torque, is_local);
}

void NodeElastoChronoD::set_fixed(bool is_fixed) {
    chobj->SetFixed(is_fixed);
}

bool NodeElastoChronoD::is_fixed() const {
    return chobj->IsFixed();
}

void NodeElastoChronoD::set_position(const Vector3d& position) {
    chobj->SetPos(vec2ch(position));
}

Vector3d NodeElastoChronoD::get_position() const {
    return ch2vec(chobj->GetPos());
}

void NodeElastoChronoD::set_velocity(const Vector3d& velocity) {
    chobj->SetPos_dt(vec2ch(velocity));
}

Vector3d NodeElastoChronoD::get_velocity() const {
    return ch2vec(chobj->GetPos_dt());
}

void NodeElastoChronoD::set_acceleration(const Vector3d& acceleration) {
    chobj->SetPos_dtdt(vec2ch(acceleration));
}

Vector3d NodeElastoChronoD::get_acceleration() const {
    return ch2vec(chobj->GetPos_dtdt());
}

void NodeElastoChronoD::set_rotational_velocity(const Vector3d& rotational_velocity, bool is_local) {
    // no rotational velocity on ChNodeFEAxyzD
}

Vector3d NodeElastoChronoD::get_rotational_velocity(bool is_local) const {
    // no rotational velocity on ChNodeFEAxyzD
    return Vector3d(0.0, 0.0, 0.0);
}

void NodeElastoChronoD::set_rotational_acceleration(const Vector3d& rotational_acceleration, bool is_local) {
    // no rotational acceleration on ChNodeFEAxyzD
}

Vector3d NodeElastoChronoD::get_rotational_acceleration(bool is_local) const {
    // no rotational acceleration on ChNodeFEAxyzD
    return Vector3d(0.0, 0.0, 0.0);
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
    chobj = chrono_types::make_shared<chrono::fea::ChElementCableANCF>();
    ElementElastoChrono::chobj = chobj;
}

void ElementMooringElastoChrono::set_nodes(std::shared_ptr<NodeElasto> node1, std::shared_ptr<NodeElasto> node2) {
    nodes.clear();
    nodes.push_back(node1);
    nodes.push_back(node2);

    // set nodes
    chobj->SetNodes(std::dynamic_pointer_cast<NodeElastoChronoD>(node1)->chobj,
                    std::dynamic_pointer_cast<NodeElastoChronoD>(node2)->chobj);
}

void ElementMooringElastoChrono::set_properties(double density,
                                                double diameter,
                                                double stiffness_axial,
                                                double stiffness_bending) {
    // create mooring section
    auto section = chrono_types::make_shared<chrono::fea::ChBeamSectionCable>();
    chobj->SetSection(section);
    section->SetDensity(density);
    double area = chrono::CH_C_PI * pow(diameter, 2) / 4.0;
    section->SetDiameter(diameter);
    double young_modulus = stiffness_axial / area;
    section->SetYoungModulus(young_modulus);
    section->SetI(stiffness_bending / young_modulus);
}

void ElementMooringElastoChrono::set_rest_length(double rest_length) {
    chobj->SetRestLength(rest_length);
}

double ElementMooringElastoChrono::get_rest_length() const {
    return chobj->GetRestLength();
}

SpringLinearChrono::SpringLinearChrono() {
    chobj = chrono_types::make_shared<chrono::ChLinkTSDA>();
}

void SpringLinearChrono::initialize(const BodyElasto& body1, const BodyElasto& body2) {
    chobj->Initialize(dynamic_cast<const BodyElastoChrono&>(body1).chobj,
                      dynamic_cast<const BodyElastoChrono&>(body2).chobj, true, chrono::ChVector<double>(0.0, 0.0, 0.0),
                      chrono::ChVector<double>(0.0, 0.0, 0.0));
}

void SpringLinearChrono::initialize_with_anchors(const BodyElasto& body1,
                                                 const BodyElasto& body2,
                                                 bool local,
                                                 const Vector3d& anchor1,
                                                 const Vector3d& anchor2) {
    chobj->Initialize(dynamic_cast<const BodyElastoChrono&>(body1).chobj,
                      dynamic_cast<const BodyElastoChrono&>(body2).chobj, local, vec2ch(anchor1), vec2ch(anchor2));
}

void SpringLinearChrono::set_rest_length(double rest_length) {
    chobj->SetRestLength(rest_length);
}

void SpringLinearChrono::set_spring_coefficient(double spring_coefficient) {
    chobj->SetSpringCoefficient(spring_coefficient);
}

void SpringLinearChrono::set_damping_coefficient(double damping_coefficient) {
    chobj->SetDampingCoefficient(damping_coefficient);
}

LinkChrono::LinkChrono() {
    chobj = chrono_types::make_shared<chrono::ChLinkMateGeneric>();
    chobj->SetConstrainedCoords(true, true, true, true, true, true);
    LinkChronoBase::chobj = chobj;
}

void LinkChrono::initialize(const Entity& entity1, const Entity& entity2) {
    try {
        chobj->Initialize(dynamic_cast<const EntityDynamicChrono&>(entity1).chobj,
                          dynamic_cast<const EntityDynamicChrono&>(entity2).chobj,
                          *dynamic_cast<const EntityDynamicChrono&>(entity2).chobj);
    } catch (const std::bad_cast& e) {
        throw std::runtime_error("Cannot link these entities.");
    }
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

LinkChronoCable::LinkChronoCable() {}

void LinkChronoCable::initialize(const Entity& entity1, const Entity& entity2) {
    try {
        auto link = chrono_types::make_shared<chrono::fea::ChLinkPointFrame>();
        auto node = dynamic_cast<const NodeElastoChronoD&>(entity1);
        auto body = dynamic_cast<const BodyElastoChrono&>(entity2);
        link->Initialize(node.chobj, body.chobj);
        chobj = link;
        LinkChronoBase::chobj = chobj;
        return;
    } catch (const std::bad_cast& e) {
    }
    try {
        auto link = chrono_types::make_shared<chrono::fea::ChLinkPointFrame>();
        auto body = dynamic_cast<const BodyElastoChrono&>(entity1);
        auto node = dynamic_cast<const NodeElastoChronoD&>(entity2);
        link->Initialize(node.chobj, body.chobj);
        chobj = link;
        LinkChronoBase::chobj = chobj;
        return;
    } catch (const std::bad_cast& e) {
    }
    try {
        auto link = chrono_types::make_shared<chrono::fea::ChLinkPointPoint>();
        auto node1 = dynamic_cast<const NodeElastoChronoD&>(entity1);
        auto node2 = dynamic_cast<const NodeElastoChronoD&>(entity2);
        link->Initialize(node1.chobj, node2.chobj);
        chobj = link;
        LinkChronoBase::chobj = chobj;
        return;
    } catch (const std::bad_cast& e) {
    }
    throw std::runtime_error("Cannot link these entities with cable link.");
}

void LinkChronoCable::set_constraints(bool surge, bool sway, bool heave, bool roll, bool pitch, bool yaw) {
    if (surge != false || sway != false || heave != false || roll != true || pitch != true || yaw != true) {
        throw std::runtime_error("Cable links can only have spherical joint constraints.");
    }
}

Vector3d LinkChronoCable::get_reaction_force() const {
    return ch2vec(chobj->Get_react_force());
}

Vector3d LinkChronoCable::get_reaction_torque() const {
    return ch2vec(chobj->Get_react_torque());
}

LinkMatrixStiffnessDampingChrono::LinkMatrixStiffnessDampingChrono() {
    // empty stiffness and damping matrices
    stiffness_matrix = Eigen::Matrix<double, 6, 6>::Zero();
    damping_matrix = Eigen::Matrix<double, 6, 6>::Zero();
}

void LinkMatrixStiffnessDampingChrono::initialize(const Entity& entity1, const Entity& entity2) {
    try {
        // cast to Chrono bodies
        auto body1 = dynamic_cast<const BodyElastoChrono&>(entity1);
        auto body2 = dynamic_cast<const BodyElastoChrono&>(entity2);

        // instantiate Chrono object
        chobj = chrono_types::make_shared<chrono::ChLoadBodyBodyBushingGeneric>(
            body1.chobj, body2.chobj, body2.chobj->GetFrame_COG_to_abs(), stiffness_matrix, damping_matrix);
    } catch (const std::bad_cast& e) {
        throw std::runtime_error("Cannot link these entities with cable link.");
    }
};

void LinkMatrixStiffnessDampingChrono::set_stiffness_matrix(const Eigen::Matrix<double, 6, 6>& stiffness_matrix) {
    this->stiffness_matrix = stiffness_matrix;
    if (chobj) {
        chobj->SetStiffnessMatrix(stiffness_matrix);
        auto kk = chobj->GetStiffnessMatrix();
    }
}

void LinkMatrixStiffnessDampingChrono::set_damping_matrix(const Eigen::Matrix<double, 6, 6>& damping_matrix) {
    this->damping_matrix = damping_matrix;
    if (chobj) {
        chobj->SetDampingMatrix(damping_matrix);
    }
}

MeshElastoChrono::MeshElastoChrono() {
    chobj = chrono_types::make_shared<chrono::fea::ChMesh>();
}

void MeshElastoChrono::add(NodeElasto& node) {
    chobj->AddNode(dynamic_cast<NodeElastoChronoBase&>(node).chobj);
}

void MeshElastoChrono::add(ElementElasto& element) {
    chobj->AddElement(dynamic_cast<ElementElastoChrono&>(element).chobj);
}

SystemElastoChrono::SystemElastoChrono() {
    chobj = chrono_types::make_shared<chrono::ChSystemSMC>();
    set_gravitational_acceleration(Vector3d(0.0, 0.0, -9.81));

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

void SystemElastoChrono::assemble() {
    spdlog::debug("Assembly of system.");
    if (is_assembled) {
        throw std::runtime_error("Component already assembled: " + std::string(typeid(*this).name()) + ".");
    }
    for (auto& turbine : turbines) {
        turbine->assemble(*this);
    }
    for (auto& component : components) {
        component->assemble(*this);
    }
    is_assembled = true;

    // needed for some Chrono (e.g. for moorings or HydroChrono floater)
    chobj->Update();

    spdlog::debug("Finished assembly of system.");
}

void SystemElastoChrono::presetup(double fraction) {
    for (auto& turbine : turbines) {
        turbine->presetup(fraction);
    }
    for (auto& component : components) {
        component->presetup(fraction);
    }
}

void SystemElastoChrono::step(double dt) {
    chobj->DoStepDynamics(dt);
}

double SystemElastoChrono::get_time() const {
    return chobj->GetChTime();
}

void SystemElastoChrono::set_time(double time) {
    chobj->SetChTime(time);
}

void SystemElastoChrono::do_statics(bool linear, int nonlinear_steps) {
    // assemble system if it was not already
    if (!is_assembled) {
        assemble();
    }

    // constrain rotor and tower
    std::vector<bool> tower_fixed;
    for (auto& turbine : turbines) {
        // rotor
        turbine->rna.link_shaft_hub->set_constraints(true, true, true, true, true, true);
        // tower
        tower_fixed.push_back(turbine->tower.nodes.front()->is_fixed());
        turbine->tower.nodes.front()->set_fixed(true);
    }

    // linear statics
    if (linear) {
        chobj->DoStaticLinear();
    }
    // nonlinear statics
    if (nonlinear_steps > 0) {
        chobj->DoStaticNonlinear(nonlinear_steps, true);
    }

    // unconstrain rotor (and tower if it was free)
    int idx_turbine = 0;
    for (auto& turbine : turbines) {
        // rotor
        turbine->rna.link_shaft_hub->set_constraints(true, true, true, false, true, true);
        // tower
        turbine->tower.nodes.front()->set_fixed(tower_fixed[idx_turbine]);
    }

    spdlog::debug("Performed statics prestep with linear step as {} and {} nonlinear steps.", linear, nonlinear_steps);
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
    chobj->Add(dynamic_cast<LinkChronoBase&>(link).chobj);
}

void SystemElastoChrono::add(LinkMatrixStiffnessDamping& link) {
    auto load_container = chrono_types::make_shared<chrono::ChLoadContainer>();
    load_container->Add(dynamic_cast<LinkMatrixStiffnessDampingChrono&>(link).chobj);
    chobj->Add(load_container);
}

void SystemElastoChrono::add(SpringLinear& spring) {
    chobj->Add(dynamic_cast<SpringLinearChrono&>(spring).chobj);
}

}  // namespace elasto
}  // namespace seahowl
