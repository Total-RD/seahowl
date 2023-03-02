#pragma once

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

chrono::ChVector<double> vec2ch(Vector3d vector_in);
Vector3d ch2vec(chrono::ChVector<double> vector_in);
chrono::ChQuaternion<double> quat2ch(Quaternion quaternion_in);
Quaternion ch2quat(chrono::ChQuaternion<double> quaternion_in);

class RigidBodyChrono : public RigidBody {
  public:
    std::shared_ptr<chrono::ChBody> chobj;
    RigidBodyChrono();
    virtual void set_position(Vector3d position) override;
    virtual void set_mass(double mass) override;
    virtual void set_inertia_diagonal(Vector3d inertia) override;
    virtual void set_rotation(Quaternion rotation) override;
    virtual void reset_forces() override;
    virtual void accumulate_torque(Vector3d torque, bool is_local) override;
    virtual double get_mass() override;
    virtual Vector3d get_position() const override;
    virtual Vector3d get_velocity() const override;
    virtual Vector3d get_acceleration() const override;
    virtual Quaternion get_rotation() const override;
    virtual Vector3d get_direction() const override;
    virtual Vector3d get_rotational_velocity_local() const override;
    virtual Vector3d get_rotational_acceleration_local() const override;
    virtual Vector3d get_rotational_velocity_global() const override;
    virtual Vector3d get_rotational_acceleration_global() const override;
};

class NodeFEAChrono : public NodeFEA {
  public:
    std::shared_ptr<chrono::fea::ChNodeFEAxyzrot> chobj;
    std::shared_ptr<chrono::fea::ChBeamSectionTimoshenkoAdvancedGeneric> section;
    NodeFEAChrono(Vector3d position, Quaternion rotation);
    virtual void set_position(Vector3d position) override;
    virtual void set_rotation(Quaternion rotation) override;
    virtual void set_load(Vector3d force) override;
    virtual void set_torque(Vector3d torque) override;
    virtual Vector3d get_position() const override;
    virtual Vector3d get_velocity() const override;
    virtual Vector3d get_acceleration() const override;
    virtual Quaternion get_rotation() const override;
    virtual Vector3d get_direction() const override;
    virtual Vector3d get_rotational_velocity_local() const override;
    virtual Vector3d get_rotational_acceleration_local() const override;
    virtual Vector3d get_rotational_velocity_global() const override;
    virtual Vector3d get_rotational_acceleration_global() const override;
    virtual Vector3d get_load() const override;
    virtual Vector3d get_torque() const override;
    void set_properties(const BladeReferencePointElasto& ref, bool fpm = false);
    void set_properties(const TowerReferencePointElasto& ref);
};

class ElementFEAChrono {
  public:
    std::shared_ptr<chrono::fea::ChElementBeam> chobj_base;
};

class BladeElementFEAChrono : public ElementFEAChrono, public BladeElementFEA {
  public:
    std::shared_ptr<chrono::fea::ChElementBeamTaperedTimoshenko> chobj;
    BladeElementFEAChrono();
    virtual void set_nodes(std::shared_ptr<NodeFEA> node1, std::shared_ptr<NodeFEA> node2) override;
    virtual void set_prebend(const Quaternion& prebend) override;
    virtual double get_mass() override;
    virtual void evaluate_position_rotation(double eta, Vector3d& position, Quaternion& rotation) override;
};

class BladeElementFEAChronoFPM : public ElementFEAChrono, public BladeElementFEA {
  public:
    std::shared_ptr<chrono::fea::ChElementBeamTaperedTimoshenkoFPM> chobj;
    BladeElementFEAChronoFPM();
    virtual void set_nodes(std::shared_ptr<NodeFEA> node1, std::shared_ptr<NodeFEA> node2) override;
    virtual void set_prebend(const Quaternion& prebend) override;
    virtual double get_mass() override;
    virtual void evaluate_position_rotation(double eta, Vector3d& position, Quaternion& rotation) override;
};

class MooringElementFEAChrono : public ElementFEAChrono, public ElementFEA {
  public:
    std::shared_ptr<chrono::fea::ChElementBeamEuler> chobj;
    MooringElementFEAChrono();
    virtual void set_nodes(std::shared_ptr<NodeFEA> node1, std::shared_ptr<NodeFEA> node2) override;
    virtual double get_mass() override;
    virtual void evaluate_position_rotation(double eta, Vector3d& position, Quaternion& rotation) override;
    void set_properties(double density, double diameter, double stiffness_axial);
};

class LinkFixChrono : public LinkFix {
  public:
    std::shared_ptr<chrono::ChLinkMateFix> chobj;
    LinkFixChrono();
    virtual void initialize(std::shared_ptr<RigidBody> body1, std::shared_ptr<RigidBody> body2) override;
    virtual void initialize(std::shared_ptr<NodeFEA> node1, std::shared_ptr<RigidBody> body2) override;
    Vector3d get_reaction_force() const override;
    Vector3d get_reaction_torque() const override;
};

class LinkRevoluteChrono : public LinkRevolute {
  public:
    std::shared_ptr<chrono::ChLinkRevolute> chobj;
    LinkRevoluteChrono();
    virtual void initialize(std::shared_ptr<RigidBody> body1, std::shared_ptr<RigidBody> body2) override;
    Vector3d get_reaction_force() const override;
    Vector3d get_reaction_torque() const override;
};

class MeshElastoChrono : public MeshElasto {
  public:
    std::shared_ptr<chrono::fea::ChMesh> chobj;
    MeshElastoChrono();
    virtual void add(std::shared_ptr<NodeFEA> node) override;
    virtual void add(std::shared_ptr<ElementFEA> element) override;
    void add(std::shared_ptr<chrono::fea::ChElementBeam> element);
};

class SystemElastoChrono : public SystemElasto {
  public:
    chrono::ChSystemSMC chobj;
    SystemElastoChrono();
    virtual Vector3d get_gravitational_acceleration() const override;
    virtual void set_gravitational_acceleration(Vector3d gravitational_acceleration) override;
    virtual void add(std::shared_ptr<RigidBody> body) override;
    virtual void add(std::shared_ptr<MeshElasto> mesh) override;
    virtual void add(std::shared_ptr<LinkFix> link) override;
    virtual void add(std::shared_ptr<LinkRevolute> link) override;
};

}  // namespace elasto
}  // namespace seahowl
