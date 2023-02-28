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

class RigidBody {
  public:
    virtual void set_position(Vector3d position) = 0;
    virtual void set_mass(double mass) = 0;
    virtual void set_inertia_diagonal(Vector3d inertia) = 0;
    virtual void set_rotation(Quaternion rotation) = 0;
    virtual void reset_forces() = 0;
    virtual void accumulate_torque(Vector3d torque, bool is_local) = 0;
    virtual double get_mass() = 0;
    virtual Vector3d get_position() const = 0;
    virtual Vector3d get_velocity() const = 0;
    virtual Vector3d get_acceleration() const = 0;
    virtual Quaternion get_rotation() const = 0;
    virtual Vector3d get_direction() const = 0;
    virtual Vector3d get_rotational_velocity_local() const = 0;
    virtual Vector3d get_rotational_acceleration_local() const = 0;
    virtual Vector3d get_rotational_velocity_global() const = 0;
    virtual Vector3d get_rotational_acceleration_global() const = 0;
};

class NodeFEA {
  public:
    virtual void set_position(Vector3d position) = 0;
    virtual void set_rotation(Quaternion rotation) = 0;
    virtual void set_load(Vector3d force) = 0;
    virtual void set_torque(Vector3d torque) = 0;
    virtual Vector3d get_position() const = 0;
    virtual Vector3d get_velocity() const = 0;
    virtual Vector3d get_acceleration() const = 0;
    virtual Quaternion get_rotation() const = 0;
    virtual Vector3d get_direction() const = 0;
    virtual Vector3d get_rotational_velocity_local() const = 0;
    virtual Vector3d get_rotational_acceleration_local() const = 0;
    virtual Vector3d get_rotational_velocity_global() const = 0;
    virtual Vector3d get_rotational_acceleration_global() const = 0;
    virtual Vector3d get_load() const = 0;
    virtual Vector3d get_torque() const = 0;
};

class ElementFEA {
  public:
    std::vector<std::shared_ptr<NodeFEA>> nodes0;

    virtual void set_nodes(std::shared_ptr<NodeFEA> node1, std::shared_ptr<NodeFEA> node2) = 0;
    virtual double get_mass() = 0;

    virtual void evaluate_position_rotation(double eta, Vector3d& position, Quaternion& rotation) = 0;
};

class BladeElementFEA : public ElementFEA {
  public:
    virtual void set_prebend(const Quaternion& prebend) = 0;
};

class RigidBodyChrono : public RigidBody, public chrono::ChBody {
  public:
    RigidBodyChrono() : chrono::ChBody(){};
    virtual void set_position(Vector3d position) override { this->SetPos(position); };
    virtual void set_mass(double mass) override { this->SetMass(mass); };
    virtual void set_inertia_diagonal(Vector3d inertia) override { this->SetInertiaXX(inertia); };
    virtual void set_rotation(Quaternion rotation) override { this->SetRot(rotation); };
    virtual void reset_forces() override { this->Empty_forces_accumulators(); };
    virtual void accumulate_torque(Vector3d torque, bool is_local) override {
        this->Accumulate_torque(torque, is_local);
    };
    virtual double get_mass() override { return this->GetMass(); };
    virtual Vector3d get_position() const override { return this->GetPos(); };
    virtual Vector3d get_velocity() const override { return this->GetPos_dt(); };
    virtual Vector3d get_acceleration() const override { return this->GetPos_dtdt(); };
    virtual Quaternion get_rotation() const override { return this->GetRot(); };
    virtual Vector3d get_direction() const override { return this->GetRot().GetVector(); };
    virtual Vector3d get_rotational_velocity_local() const override { return this->GetWvel_loc(); };
    virtual Vector3d get_rotational_acceleration_local() const override { return this->GetWacc_loc(); };
    virtual Vector3d get_rotational_velocity_global() const override { return this->GetWvel_par(); };
    virtual Vector3d get_rotational_acceleration_global() const override { return this->GetWacc_par(); };
};

class NodeFEAChrono : public NodeFEA, public chrono::fea::ChNodeFEAxyzrot {
  public:
    std::shared_ptr<chrono::fea::ChBeamSectionTimoshenkoAdvancedGeneric> section;

    NodeFEAChrono(Vector3d position, Quaternion rotation)
        : chrono::fea::ChNodeFEAxyzrot(chrono::ChFrame<>(position, rotation)){};

    virtual void set_position(Vector3d position) override { this->SetPos(position); };
    virtual void set_rotation(Quaternion rotation) override { this->SetRot(rotation); };
    virtual void set_load(Vector3d force) override { this->SetForce(force); };
    virtual void set_torque(Vector3d torque) override { this->SetTorque(torque); };
    virtual Vector3d get_position() const override { return Vector3d(this->GetPos()); };
    virtual Vector3d get_velocity() const override { return Vector3d(this->GetPos_dt()); };
    virtual Vector3d get_acceleration() const override { return Vector3d(this->GetPos_dtdt()); };
    virtual Quaternion get_rotation() const override { return Quaternion(this->GetRot()); };
    virtual Vector3d get_direction() const override {
        return Vector3d(this->TransformDirectionLocalToParent(Vector3d(1.0, 0.0, 0.0)));
    };
    virtual Vector3d get_rotational_velocity_local() const override { return Vector3d(this->GetWvel_loc()); };
    virtual Vector3d get_rotational_acceleration_local() const override { return Vector3d(this->GetWacc_loc()); };
    virtual Vector3d get_rotational_velocity_global() const override { return Vector3d(this->GetWvel_par()); };
    virtual Vector3d get_rotational_acceleration_global() const override { return Vector3d(this->GetWacc_par()); };
    virtual Vector3d get_load() const override { return Vector3d(this->GetForce()); };
    virtual Vector3d get_torque() const override { return Vector3d(this->GetTorque()); };
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

class BladeElementFEAChrono : public BladeElementFEA, public chrono::fea::ChElementBeamTaperedTimoshenko {
  public:
    BladeElementFEAChrono() : chrono::fea::ChElementBeamTaperedTimoshenko() {
        // create blade section
        auto blade_section = chrono_types::make_shared<chrono::fea::ChBeamSectionTaperedTimoshenkoAdvancedGeneric>();
        this->SetTaperedSection(blade_section);
    };

    virtual void set_nodes(std::shared_ptr<NodeFEA> node1, std::shared_ptr<NodeFEA> node2) override {
        nodes0.clear();
        nodes0.push_back(node1);
        nodes0.push_back(node2);

        // set nodes
        auto ch1 = std::dynamic_pointer_cast<chrono::fea::ChNodeFEAxyzrot>(node1);
        auto ch2 = std::dynamic_pointer_cast<chrono::fea::ChNodeFEAxyzrot>(node2);
        this->SetNodes(ch1, ch2);
        // set tapered sections
        this->GetTaperedSection()->SetSectionA(std::dynamic_pointer_cast<NodeFEAChrono>(node1)->section);
        this->GetTaperedSection()->SetSectionB(std::dynamic_pointer_cast<NodeFEAChrono>(node2)->section);
    };

    virtual void set_prebend(const Quaternion& prebend) override { this->SetNodeBreferenceRot(prebend); };

    virtual double get_mass() override { return this->GetMass(); };

    virtual void evaluate_position_rotation(double eta, Vector3d& position, Quaternion& rotation) override {
        this->EvaluateSectionFrame(eta, position, rotation);
    };
};

class LinkFix : public chrono::ChLinkMateFix {
  public:
    LinkFix() : chrono::ChLinkMateFix(){};
    void initialize(std::shared_ptr<RigidBody> body1, std::shared_ptr<RigidBody> body2) {
        this->Initialize(std::dynamic_pointer_cast<RigidBodyChrono>(body1),
                         std::dynamic_pointer_cast<RigidBodyChrono>(body2));
    };
    void initialize(std::shared_ptr<NodeFEA> node1, std::shared_ptr<RigidBody> body2) {
        this->Initialize(std::dynamic_pointer_cast<NodeFEAChrono>(node1),
                         std::dynamic_pointer_cast<RigidBodyChrono>(body2));
    };
};

class LinkRevolute : public chrono::ChLinkRevolute {
  public:
    LinkRevolute() : chrono::ChLinkRevolute(){};
    void initialize(std::shared_ptr<RigidBody> body1, std::shared_ptr<RigidBody> body2) {
        this->Initialize(std::dynamic_pointer_cast<RigidBodyChrono>(body1),
                         std::dynamic_pointer_cast<RigidBodyChrono>(body2),
                         std::dynamic_pointer_cast<RigidBodyChrono>(body2)->GetFrame_COG_to_abs());
    };
};

class MeshElasto : public chrono::fea::ChMesh {
  public:
    MeshElasto() : chrono::fea::ChMesh(){};
    void add(std::shared_ptr<NodeFEA> node) { this->AddNode(std::dynamic_pointer_cast<NodeFEAChrono>(node)); };
    void add(std::shared_ptr<ElementFEA> element) {
        this->AddElement(std::dynamic_pointer_cast<BladeElementFEAChrono>(element));
    };
    void add(std::shared_ptr<chrono::fea::ChElementBeam> element) { this->AddElement(element); };
};

class SystemElasto : public chrono::ChSystemSMC {
  public:
    SystemElasto() : chrono::ChSystemSMC(){};
    void add(std::shared_ptr<RigidBody> body) { this->Add(std::dynamic_pointer_cast<RigidBodyChrono>(body)); };
    void add(std::shared_ptr<MeshElasto> mesh) { this->Add(std::dynamic_pointer_cast<MeshElasto>(mesh)); };
    void add(std::shared_ptr<LinkFix> link) { this->Add(link); };
    void add(std::shared_ptr<LinkRevolute> link) { this->Add(link); };
};

}  // namespace elasto
}  // namespace seahowl
