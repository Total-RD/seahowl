#pragma once

#include <chrono/core/ChVector.h>
#include <chrono/core/ChMatrix.h>
#include <chrono/fea/ChBeamSectionTaperedTimoshenkoFPM.h>
#include <chrono/fea/ChElementBeamTaperedTimoshenkoFPM.h>
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

chrono::ChVector<double> vec2ch(Vector3d vector_in);
Vector3d ch2vec(chrono::ChVector<double> vector_in);
chrono::ChQuaternion<double> quat2ch(Quaternion quaternion_in);
Quaternion ch2quat(chrono::ChQuaternion<double> quaternion_in);

class RigidBodyChrono : public RigidBody {
  public:
    std::shared_ptr<chrono::ChBody> chobj;
    RigidBodyChrono() { chobj = chrono_types::make_shared<chrono::ChBody>(); };
    virtual void set_position(Vector3d position) override { chobj->SetPos(vec2ch(position)); };
    virtual void set_mass(double mass) override { chobj->SetMass(mass); };
    virtual void set_inertia_diagonal(Vector3d inertia) override { chobj->SetInertiaXX(vec2ch(inertia)); };
    virtual void set_rotation(Quaternion rotation) override { chobj->SetRot(quat2ch(rotation)); };
    virtual void reset_forces() override { chobj->Empty_forces_accumulators(); };
    virtual void accumulate_torque(Vector3d torque, bool is_local) override {
        chobj->Accumulate_torque(torque, is_local);
    };
    virtual double get_mass() override { return chobj->GetMass(); };
    virtual Vector3d get_position() const override { return ch2vec(chobj->GetPos()); };
    virtual Vector3d get_velocity() const override { return ch2vec(chobj->GetPos_dt()); };
    virtual Vector3d get_acceleration() const override { return ch2vec(chobj->GetPos_dtdt()); };
    virtual Quaternion get_rotation() const override { return ch2quat(chobj->GetRot()); };
    virtual Vector3d get_direction() const override { return ch2vec(chobj->GetRot().GetVector()); };
    virtual Vector3d get_rotational_velocity_local() const override { return ch2vec(chobj->GetWvel_loc()); };
    virtual Vector3d get_rotational_acceleration_local() const override { return ch2vec(chobj->GetWacc_loc()); };
    virtual Vector3d get_rotational_velocity_global() const override { return ch2vec(chobj->GetWvel_par()); };
    virtual Vector3d get_rotational_acceleration_global() const override { return ch2vec(chobj->GetWacc_par()); };
};

class NodeFEAChrono : public NodeFEA {
  public:
    std::shared_ptr<chrono::fea::ChNodeFEAxyzrot> chobj;
    std::shared_ptr<chrono::fea::ChBeamSectionTimoshenkoAdvancedGeneric> section;

    NodeFEAChrono(Vector3d position, Quaternion rotation) {
        chobj = chrono_types::make_shared<chrono::fea::ChNodeFEAxyzrot>(
            chrono::ChFrame<>(vec2ch(position), quat2ch(rotation)));
    };

    virtual void set_position(Vector3d position) override { chobj->SetPos(vec2ch(position)); };
    virtual void set_rotation(Quaternion rotation) override { chobj->SetRot(quat2ch(rotation)); };
    virtual void set_load(Vector3d force) override { chobj->SetForce(vec2ch(force)); };
    virtual void set_torque(Vector3d torque) override { chobj->SetTorque(vec2ch(torque)); };
    virtual Vector3d get_position() const override { return ch2vec(chobj->GetPos()); };
    virtual Vector3d get_velocity() const override { return ch2vec(chobj->GetPos_dt()); };
    virtual Vector3d get_acceleration() const override { return ch2vec(chobj->GetPos_dtdt()); };
    virtual Quaternion get_rotation() const override { return Quaternion(ch2quat(chobj->GetRot())); };
    virtual Vector3d get_direction() const override {
        return ch2vec(chobj->TransformDirectionLocalToParent(chrono::ChVector<double>(1.0, 0.0, 0.0)));
    };
    virtual Vector3d get_rotational_velocity_local() const override { return ch2vec(chobj->GetWvel_loc()); };
    virtual Vector3d get_rotational_acceleration_local() const override { return ch2vec(chobj->GetWacc_loc()); };
    virtual Vector3d get_rotational_velocity_global() const override { return ch2vec(chobj->GetWvel_par()); };
    virtual Vector3d get_rotational_acceleration_global() const override { return ch2vec(chobj->GetWacc_par()); };
    virtual Vector3d get_load() const override { return ch2vec(chobj->GetForce()); };
    virtual Vector3d get_torque() const override { return ch2vec(chobj->GetTorque()); };
    void set_properties(const BladeReferencePointElasto& ref) {
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
    };
    void set_properties(const TowerReferencePointElasto& ref) {
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
    };
};

class BladeElementFEAChrono : public BladeElementFEA {
  public:
    std::shared_ptr<chrono::fea::ChElementBeamTaperedTimoshenko> chobj;
    BladeElementFEAChrono() {
        chobj = chrono_types::make_shared<chrono::fea::ChElementBeamTaperedTimoshenko>();
        // create blade section
        auto blade_section = chrono_types::make_shared<chrono::fea::ChBeamSectionTaperedTimoshenkoAdvancedGeneric>();
        chobj->SetTaperedSection(blade_section);
    };

    virtual void set_nodes(std::shared_ptr<NodeFEA> node1, std::shared_ptr<NodeFEA> node2) override {
        nodes0.clear();
        nodes0.push_back(node1);
        nodes0.push_back(node2);

        // set nodes
        chobj->SetNodes(std::dynamic_pointer_cast<NodeFEAChrono>(node1)->chobj,
                        std::dynamic_pointer_cast<NodeFEAChrono>(node2)->chobj);
        // set tapered sections
        chobj->GetTaperedSection()->SetSectionA(std::dynamic_pointer_cast<NodeFEAChrono>(node1)->section);
        chobj->GetTaperedSection()->SetSectionB(std::dynamic_pointer_cast<NodeFEAChrono>(node2)->section);
    };

    virtual void set_prebend(const Quaternion& prebend) override { chobj->SetNodeBreferenceRot(quat2ch(prebend)); };

    virtual double get_mass() override { return chobj->GetMass(); };

    virtual void evaluate_position_rotation(double eta, Vector3d& position, Quaternion& rotation) override {
        auto chvec = vec2ch(position);
        auto chquat = quat2ch(rotation);
        chobj->EvaluateSectionFrame(eta, chvec, chquat);
        position[0] = chvec[0];
        position[1] = chvec[1];
        position[2] = chvec[2];
        rotation = ch2quat(chquat);
    };
};

class LinkFixChrono : public LinkFix {
  public:
    std::shared_ptr<chrono::ChLinkMateFix> chobj;
    LinkFixChrono() { chobj = chrono_types::make_shared<chrono::ChLinkMateFix>(); };
    virtual void initialize(std::shared_ptr<RigidBody> body1, std::shared_ptr<RigidBody> body2) override {
        chobj->Initialize(std::dynamic_pointer_cast<RigidBodyChrono>(body1)->chobj,
                          std::dynamic_pointer_cast<RigidBodyChrono>(body2)->chobj);
    };
    virtual void initialize(std::shared_ptr<NodeFEA> node1, std::shared_ptr<RigidBody> body2) override {
        chobj->Initialize(std::dynamic_pointer_cast<NodeFEAChrono>(node1)->chobj,
                          std::dynamic_pointer_cast<RigidBodyChrono>(body2)->chobj);
    };
    Vector3d get_reaction_force() const override { return ch2vec(chobj->Get_react_force()); };
    Vector3d get_reaction_torque() const override { return ch2vec(chobj->Get_react_torque()); };
};

class LinkRevoluteChrono : public LinkRevolute {
  public:
    std::shared_ptr<chrono::ChLinkRevolute> chobj;
    LinkRevoluteChrono() { chobj = chrono_types::make_shared<chrono::ChLinkRevolute>(); };
    virtual void initialize(std::shared_ptr<RigidBody> body1, std::shared_ptr<RigidBody> body2) override {
        chobj->Initialize(std::dynamic_pointer_cast<RigidBodyChrono>(body1)->chobj,
                          std::dynamic_pointer_cast<RigidBodyChrono>(body2)->chobj,
                          std::dynamic_pointer_cast<RigidBodyChrono>(body2)->chobj->GetFrame_COG_to_abs());
    };
    Vector3d get_reaction_force() const override { return ch2vec(chobj->Get_react_force()); };
    Vector3d get_reaction_torque() const override { return ch2vec(chobj->Get_react_torque()); };
};

class MeshElastoChrono : public MeshElasto {
  public:
    std::shared_ptr<chrono::fea::ChMesh> chobj;
    MeshElastoChrono() { chobj = chrono_types::make_shared<chrono::fea::ChMesh>(); };
    virtual void add(std::shared_ptr<NodeFEA> node) override {
        chobj->AddNode(std::dynamic_pointer_cast<NodeFEAChrono>(node)->chobj);
    };
    virtual void add(std::shared_ptr<ElementFEA> element) override {
        chobj->AddElement(std::dynamic_pointer_cast<BladeElementFEAChrono>(element)->chobj);
    };
    void add(std::shared_ptr<chrono::fea::ChElementBeam> element) { chobj->AddElement(element); };
};

class SystemElastoChrono : public SystemElasto {
  public:
    chrono::ChSystemSMC chobj;
    SystemElastoChrono(){};
    virtual Vector3d get_gravitational_acceleration() const override { return ch2vec(chobj.Get_G_acc()); };
    virtual void set_gravitational_acceleration(Vector3d gravitational_acceleration) override {
        chobj.Set_G_acc(gravitational_acceleration);
    };
    virtual void add(std::shared_ptr<RigidBody> body) override {
        chobj.Add(std::dynamic_pointer_cast<RigidBodyChrono>(body)->chobj);
    };
    virtual void add(std::shared_ptr<MeshElasto> mesh) override {
        chobj.Add(std::dynamic_pointer_cast<MeshElastoChrono>(mesh)->chobj);
    };
    virtual void add(std::shared_ptr<LinkFix> link) override {
        chobj.Add(std::dynamic_pointer_cast<LinkFixChrono>(link)->chobj);
    };
    virtual void add(std::shared_ptr<LinkRevolute> link) override {
        chobj.Add(std::dynamic_pointer_cast<LinkRevoluteChrono>(link)->chobj);
    };
};

}  // namespace elasto
}  // namespace seahowl
