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

#include <vector>
#include <memory>

namespace seahowl {
namespace elasto {

class RigidBody : public chrono::ChBody {
  public:
    RigidBody() : chrono::ChBody(){};
    void set_position(Vector3d position) { this->SetPos(position); };
    void set_mass(double mass) { this->SetMass(mass); };
    double get_mass() { return this->GetMass(); };
    void set_rotation(Quaternion rotation) { this->SetRot(rotation); };
    Vector3d get_position() { return this->GetPos(); };
    Vector3d get_velocity() { return this->GetPos_dt(); };
    Vector3d get_acceleration() { return this->GetPos_dtdt(); };
    Quaternion get_rotation() { return this->GetRot(); };
    Vector3d get_direction() { return this->GetRot().GetVector(); };
    Vector3d get_rotational_velocity_local() { return this->GetWvel_loc(); };
    Vector3d get_rotational_acceleration_local() { return this->GetWacc_loc(); };
};

class NodeFEA : public chrono::fea::ChNodeFEAxyzrot {
  public:
    std::shared_ptr<chrono::fea::ChBeamSectionTimoshenkoAdvancedGeneric> section;

    NodeFEA(Vector3d position, Quaternion rotation)
        : chrono::fea::ChNodeFEAxyzrot(chrono::ChFrame<>(position, rotation)){};

    void set_position(Vector3d position) { this->SetPos(position); };
    void set_rotation(Quaternion rotation) { this->SetRot(rotation); };
    Vector3d get_position() { return Vector3d(this->GetPos()); };
    Vector3d get_velocity() { return Vector3d(this->GetPos_dt()); };
    Vector3d get_acceleration() { return Vector3d(this->GetPos_dtdt()); };
    Quaternion get_rotation() { return Quaternion(this->GetRot()); };
    Vector3d get_direction() { return Vector3d(this->TransformDirectionLocalToParent(Vector3d(1.0, 0.0, 0.0))); };
    Vector3d get_rotational_velocity_local() { return Vector3d(this->GetWvel_loc()); };
    Vector3d get_rotational_acceleration_local() { return Vector3d(this->GetWacc_loc()); };
    Vector3d get_rotational_velocity_global() { return Vector3d(this->GetWvel_par()); };
    Vector3d get_rotational_acceleration_global() { return Vector3d(this->GetWacc_par()); };
    Vector3d get_load() { return Vector3d(this->GetForce()); };
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

class BladeElementFEA : public chrono::fea::ChElementBeamTaperedTimoshenko {
  public:
    std::vector<std::shared_ptr<NodeFEA>> nodes;
    BladeElementFEA() : chrono::fea::ChElementBeamTaperedTimoshenko() {
        // create blade section
        auto blade_section = chrono_types::make_shared<chrono::fea::ChBeamSectionTaperedTimoshenkoAdvancedGeneric>();
        this->SetTaperedSection(blade_section);
    };

    void set_nodes(std::shared_ptr<NodeFEA> node1, std::shared_ptr<NodeFEA> node2) {
        nodes.clear();
        nodes.push_back(node1);
        nodes.push_back(node2);

        // set nodes
        this->SetNodes(node1, node2);
        // set tapered sections
        this->GetTaperedSection()->SetSectionA(node1->section);
        this->GetTaperedSection()->SetSectionB(node2->section);
    };

    void set_prebend(const Quaternion& prebend) { this->SetNodeBreferenceRot(prebend); };
};

class LinkFix : public chrono::ChLinkMateFix {
  public:
    LinkFix() : chrono::ChLinkMateFix(){};
    void initialize(std::shared_ptr<RigidBody> body1, std::shared_ptr<RigidBody> body2) {
        this->Initialize(body1, body2);
    };
    void initialize(std::shared_ptr<NodeFEA> node1, std::shared_ptr<RigidBody> body2) {
        this->Initialize(node1, body2);
    };
};

class LinkRevolute : public chrono::ChLinkRevolute {
  public:
    LinkRevolute() : chrono::ChLinkRevolute(){};
    void initialize(std::shared_ptr<RigidBody> body1, std::shared_ptr<RigidBody> body2) {
        this->Initialize(body1, body2, body2->GetFrame_COG_to_abs());
    };
};

class MeshElasto : public chrono::fea::ChMesh {
  public:
    MeshElasto() : chrono::fea::ChMesh(){};
    void add(std::shared_ptr<NodeFEA> node) { this->AddNode(node); };
    void add(std::shared_ptr<BladeElementFEA> element) { this->AddElement(element); };
    void add(std::shared_ptr<chrono::fea::ChElementBeam> element) { this->AddElement(element); };
};

class SystemElasto : public chrono::ChSystemSMC {
  public:
    SystemElasto() : chrono::ChSystemSMC(){};
    void add(std::shared_ptr<RigidBody> body) { this->Add(body); };
    void add(std::shared_ptr<MeshElasto> mesh) { this->Add(mesh); };
    void add(std::shared_ptr<LinkFix> link) { this->Add(link); };
    void add(std::shared_ptr<LinkRevolute> link) { this->Add(link); };
};

}  // namespace elasto
}  // namespace seahowl
