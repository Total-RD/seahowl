#pragma once

#include <seahowl/elasto/reference_point_elasto.h>
#include <seahowl/elasto/entities_elasto.h>

#include <vector>
#include <memory>

namespace chrono {
template <class Real>
class ChVector;
template <class Real>
class ChQuaternion;
template <class Real>
class ChFrameMoving;
class ChBody;
class ChLinkMateGeneric;
class ChSystem;
namespace fea {
class ChNodeFEAxyzrot;
class ChElementBeam;
class ChElementBeamEuler;
class ChElementBeamTaperedTimoshenko;
class ChElementBeamTaperedTimoshenkoFPM;
class ChBeamSectionTimoshenkoAdvancedGeneric;
class ChMesh;
}  // namespace fea
}  // namespace chrono

namespace seahowl {
namespace elasto {

chrono::ChVector<double> vec2ch(Vector3d vector_in);
Vector3d ch2vec(chrono::ChVector<double> vector_in);
chrono::ChQuaternion<double> quat2ch(Quaternion quaternion_in);
Quaternion ch2quat(chrono::ChQuaternion<double> quaternion_in);

//
class EntityDynamicChrono : public EntityDynamic {
  public:
    std::shared_ptr<chrono::ChFrameMoving<double>> chobj;

    virtual void set_position(Vector3d position) override;
    virtual Vector3d get_position() const override;
    virtual void set_rotation(Quaternion rotation) override;
    virtual Quaternion get_rotation() const override;
    virtual void set_velocity(Vector3d velocity) override;
    virtual Vector3d get_velocity() const override;
    virtual void set_acceleration(Vector3d acceleration) override;
    virtual Vector3d get_acceleration() const override;
    virtual void set_rotational_velocity(Vector3d rotational_velocity) override;
    virtual Vector3d get_rotational_velocity() const override;
    virtual void set_rotational_acceleration(Vector3d rotational_acceleration) override;
    virtual Vector3d get_rotational_acceleration() const override;
};

/**
 * @brief Chrono rigid body class.
 */
class BodyElastoChrono : public BodyElasto {
  public:
    /** @brief Pointer to underlying Chrono object. */
    std::shared_ptr<chrono::ChBody> chobj;

    BodyElastoChrono();
    virtual void set_position(Vector3d position) override;
    virtual Vector3d get_position() const override;
    virtual void set_rotation(Quaternion rotation) override;
    virtual Quaternion get_rotation() const override;
    virtual void set_velocity(Vector3d velocity) override;
    virtual Vector3d get_velocity() const override;
    virtual void set_acceleration(Vector3d acceleration) override;
    virtual Vector3d get_acceleration() const override;
    virtual void set_rotational_velocity(Vector3d rotational_velocity) override;
    virtual Vector3d get_rotational_velocity() const override;
    virtual void set_rotational_acceleration(Vector3d rotational_acceleration) override;
    virtual Vector3d get_rotational_acceleration() const override;
    virtual void set_mass(double mass) override;
    virtual void set_inertia_diagonal(Vector3d inertia) override;
    virtual void reset_forces() override;
    virtual void accumulate_torque(Vector3d torque, bool is_local) override;
    virtual void set_fixed(bool is_fixed) override;
    virtual double get_mass() override;
};

/**
 * @brief Chrono elasto node class.
 */
class NodeElastoChrono : public NodeElasto {
  public:
    /** @brief Pointer to underlying Chrono object. */
    std::shared_ptr<chrono::fea::ChNodeFEAxyzrot> chobj;
    std::shared_ptr<chrono::fea::ChBeamSectionTimoshenkoAdvancedGeneric> section;

    NodeElastoChrono(Vector3d position, Quaternion rotation);
    virtual void set_position(Vector3d position) override;
    virtual Vector3d get_position() const override;
    virtual void set_rotation(Quaternion rotation) override;
    virtual Quaternion get_rotation() const override;
    virtual void set_velocity(Vector3d velocity) override;
    virtual Vector3d get_velocity() const override;
    virtual void set_acceleration(Vector3d acceleration) override;
    virtual Vector3d get_acceleration() const override;
    virtual void set_rotational_velocity(Vector3d rotational_velocity) override;
    virtual Vector3d get_rotational_velocity() const override;
    virtual void set_rotational_acceleration(Vector3d rotational_acceleration) override;
    virtual Vector3d get_rotational_acceleration() const override;
    virtual Vector3d get_direction() const override;
    virtual void set_load(Vector3d force) override;
    virtual Vector3d get_load() const override;
    virtual void set_torque(Vector3d torque) override;
    virtual Vector3d get_torque() const override;
    virtual void set_fixed(bool is_fixed) override;
    void set_properties(const BladeReferencePointElasto& ref, bool fpm = false);
    void set_properties(const TowerReferencePointElasto& ref);
};

/**
 * @brief Chrono elasto element class.
 */
class ElementElastoChrono {
  public:
    std::shared_ptr<chrono::fea::ChElementBeam> chobj_base;
};

/**
 * @brief Chrono elasto blade element class.
 */
class ElementBladeElastoChrono : public ElementElastoChrono, public ElementBladeElasto {
  public:
    /** @brief Pointer to underlying Chrono object. */
    std::shared_ptr<chrono::fea::ChElementBeamTaperedTimoshenko> chobj;

    ElementBladeElastoChrono();
    virtual void set_nodes(std::shared_ptr<NodeElasto> node1, std::shared_ptr<NodeElasto> node2) override;
    virtual void set_prebend(const Quaternion& prebend) override;
    virtual double get_mass() override;
    virtual void evaluate_position_rotation(double eta, Vector3d& position, Quaternion& rotation) override;
};

/**
 * @brief Chrono FPM elasto blade element class.
 */
class ElementBladeElastoChronoFPM : public ElementElastoChrono, public ElementBladeElasto {
  public:
    /** @brief Pointer to underlying Chrono object. */
    std::shared_ptr<chrono::fea::ChElementBeamTaperedTimoshenkoFPM> chobj;

    ElementBladeElastoChronoFPM();
    virtual void set_nodes(std::shared_ptr<NodeElasto> node1, std::shared_ptr<NodeElasto> node2) override;
    virtual void set_prebend(const Quaternion& prebend) override;
    virtual double get_mass() override;
    virtual void evaluate_position_rotation(double eta, Vector3d& position, Quaternion& rotation) override;
};

/**
 * @brief Chrono elasto mooring element class.
 */
class ElementMooringElastoChrono : public ElementElastoChrono, public ElementMooringElasto {
  public:
    /** @brief Pointer to underlying Chrono object. */
    std::shared_ptr<chrono::fea::ChElementBeamEuler> chobj;

    ElementMooringElastoChrono();
    virtual void set_nodes(std::shared_ptr<NodeElasto> node1, std::shared_ptr<NodeElasto> node2) override;
    virtual double get_mass() override;
    virtual void evaluate_position_rotation(double eta, Vector3d& position, Quaternion& rotation) override;
    virtual void set_properties(double density, double diameter, double stiffness_axial) override;
};

/**
 * @brief Chrono link class.
 */
class LinkChrono : public Link {
  public:
    /** @brief Pointer to underlying Chrono object. */
    std::shared_ptr<chrono::ChLinkMateGeneric> chobj;

    LinkChrono();
    virtual void set_constraints(bool surge, bool sway, bool heave, bool roll, bool pitch, bool yaw) override;
    virtual void initialize(std::shared_ptr<BodyElasto> body1, std::shared_ptr<BodyElasto> body2) override;
    virtual void initialize(std::shared_ptr<NodeElasto> node1, std::shared_ptr<BodyElasto> body2) override;
    Vector3d get_reaction_force() const override;
    Vector3d get_reaction_torque() const override;
};

/**
 * @brief Chrono elasto mesh class.
 */
class MeshElastoChrono : public MeshElasto {
  public:
    /** @brief Pointer to underlying Chrono object. */
    std::shared_ptr<chrono::fea::ChMesh> chobj;

    MeshElastoChrono();
    virtual void add(std::shared_ptr<NodeElasto> node) override;
    virtual void add(std::shared_ptr<ElementElasto> element) override;
    void add(std::shared_ptr<chrono::fea::ChElementBeam> element);
};

/**
 * @brief Chrono elasto system class.
 */
class SystemElastoChrono : public SystemElasto {
  public:
    /** @brief Pointer to underlying Chrono object. */
    std::shared_ptr<chrono::ChSystem> chobj;

    SystemElastoChrono();
    virtual void step(double dt) override;
    virtual double get_time() const override;
    virtual void do_statics(bool linear, int nonlinear_steps) override;
    virtual Vector3d get_gravitational_acceleration() const override;
    virtual void set_gravitational_acceleration(Vector3d gravitational_acceleration) override;
    virtual void add(std::shared_ptr<BodyElasto> body) override;
    virtual void add(std::shared_ptr<MeshElasto> mesh) override;
    virtual void add(std::shared_ptr<Link> link) override;
};

}  // namespace elasto
}  // namespace seahowl
