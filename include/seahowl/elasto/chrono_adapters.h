#pragma once

// Disable inherits via dominance warning when there is multiple inheritance
#pragma warning(disable : 4250)

#include "seahowl/elasto/entities_elasto.h"
#include "seahowl/elasto/system_elasto.h"

#include <vector>
#include <memory>

// forward declarations
namespace seahowl {
namespace elasto {
struct BladeReferencePointElasto;
struct TowerReferencePointElasto;
}  // namespace elasto
}  // namespace seahowl

// forward declarations chrono
namespace chrono {
template <class Real>
class ChVector;
template <class Real>
class ChQuaternion;
template <class Real>
class ChFrameMoving;
class ChBody;
class ChBodyFrame;
class ChLinkBase;
class ChLinkPointPoint;
class ChLinkPointFrame;
class ChLinkMateGeneric;
class ChLinkTSDA;
class ChLoadBodyBodyBushingGeneric;
class ChSystem;
class ChLoadLocal66;
class ChLoadContainer;
namespace fea {
class ChNodeFEAbase;
class ChNodeFEAxyzrot;
class ChNodeFEAxyzD;
class ChElementBeam;
class ChElementBeamEuler;
class ChElementCableANCF;
class ChElementBeamTaperedTimoshenko;
class ChElementBeamTaperedTimoshenkoFPM;
class ChBeamSectionTimoshenkoAdvancedGeneric;
class ChMesh;
}  // namespace fea
}  // namespace chrono

namespace seahowl {
namespace elasto {

//
class EntityDynamicChrono : public virtual EntityDynamic {
  public:
    std::shared_ptr<chrono::ChBodyFrame> chobj;

    virtual void set_position(const Vector3d& position) override;
    virtual Vector3d get_position() const override;
    virtual void set_rotation(const Quaternion& rotation) override;
    virtual Quaternion get_rotation() const override;
    virtual void set_velocity(const Vector3d& velocity) override;
    virtual Vector3d get_velocity() const override;
    virtual void set_acceleration(const Vector3d& acceleration) override;
    virtual Vector3d get_acceleration() const override;
    virtual void set_rotational_velocity(const Vector3d& rotational_velocity, bool is_local = true) override;
    virtual Vector3d get_rotational_velocity(bool is_local = true) const override;
    virtual void set_rotational_acceleration(const Vector3d& rotational_acceleration, bool is_local = true) override;
    virtual Vector3d get_rotational_acceleration(bool is_local = true) const override;
};

/**
 * @brief Chrono rigid body class.
 */
class BodyElastoChrono : public BodyElasto, public EntityDynamicChrono {
  public:
    /** @brief Pointer to underlying Chrono object. */
    std::shared_ptr<chrono::ChBody> chobj;
    std::shared_ptr<chrono::ChLoadLocal66> chload66;

    BodyElastoChrono();
    virtual void set_mass(double mass) override;
    virtual void set_inertia_diagonal(const Vector3d& inertia) override;
    virtual void set_inertia_matrix(const Eigen::Matrix<double, 3, 3>& inertia) override;
    virtual Eigen::Matrix<double, 3, 3> get_inertia_matrix() const override;
    virtual void reset_loads() override;
    virtual Vector3d get_force(bool is_local = false) const override;
    virtual Vector3d get_torque(bool is_local = true) const override;
    virtual void set_force(const Vector3d& force, bool is_local = false) override;
    virtual void set_torque(const Vector3d& torque, bool is_local = true) override;
    virtual void accumulate_force(const Vector3d& force, bool is_local = false) override;
    virtual void accumulate_torque(const Vector3d& torque, bool is_local = true) override;
    virtual void set_fixed(bool is_fixed) override;
    virtual bool is_fixed() const override;
    virtual double get_mass() override;
    virtual void set_added_mass_matrix(const Eigen::Matrix<double, 6, 6>& matrix) override;
    virtual Eigen::Matrix<double, 6, 6> get_added_mass_matrix() const override;
};

class NodeElastoChronoBase {
  public:
    /** @brief Pointer to underlying Chrono object. */
    std::shared_ptr<chrono::fea::ChNodeFEAbase> chobj;
    std::shared_ptr<chrono::ChLoadLocal66> chload66;
};

/**
 * @brief Chrono elasto node class.
 */
class NodeElastoChrono : public NodeElasto, public EntityDynamicChrono, public NodeElastoChronoBase {
  public:
    /** @brief Pointer to underlying Chrono object. */
    std::shared_ptr<chrono::fea::ChNodeFEAxyzrot> chobj;
    std::shared_ptr<chrono::fea::ChBeamSectionTimoshenkoAdvancedGeneric> section;

    NodeElastoChrono(const Vector3d& position, const Quaternion& rotation);
    virtual void set_rotation(const Quaternion& rotation) override;
    virtual Quaternion get_rotation() const override;
    virtual Vector3d get_direction() const override;
    virtual void reset_loads() override;
    virtual Vector3d get_force(bool is_local = false) const override;
    virtual Vector3d get_torque(bool is_local = true) const override;
    virtual void set_force(const Vector3d& force, bool is_local = false) override;
    virtual void set_torque(const Vector3d& torque, bool is_local = true) override;
    virtual void accumulate_force(const Vector3d& force, bool is_local = false) override;
    virtual void accumulate_torque(const Vector3d& torque, bool is_local = true) override;
    virtual void set_fixed(bool is_fixed) override;
    virtual bool is_fixed() const override;
    virtual void set_mass(double mass) override;
    virtual double get_mass() override;
    virtual void set_added_mass_matrix(const Eigen::Matrix<double, 6, 6>& matrix) override;
    virtual Eigen::Matrix<double, 6, 6> get_added_mass_matrix() const override;
    void set_properties(const BladeReferencePointElasto& ref, bool fpm = false);
    void set_properties(const TowerReferencePointElasto& ref);
};

class NodeElastoChronoD : public NodeElasto, public NodeElastoChronoBase {
  public:
    /** @brief Pointer to underlying Chrono object. */
    std::shared_ptr<chrono::fea::ChNodeFEAxyzD> chobj;

    NodeElastoChronoD(const Vector3d& position, const Vector3d& direction);
    virtual void set_rotation(const Quaternion& rotation) override;
    virtual Quaternion get_rotation() const override;
    virtual Vector3d get_direction() const override;
    virtual void reset_loads() override;
    virtual Vector3d get_force(bool is_local = false) const override;
    virtual Vector3d get_torque(bool is_local = true) const override;
    virtual void set_force(const Vector3d& force, bool is_local = false) override;
    virtual void set_torque(const Vector3d& torque, bool is_local = true) override;
    virtual void accumulate_force(const Vector3d& force, bool is_local = false) override;
    virtual void accumulate_torque(const Vector3d& torque, bool is_local = true) override;
    virtual void set_fixed(bool is_fixed) override;
    virtual bool is_fixed() const override;
    virtual void set_mass(double mass) override;
    virtual double get_mass() override;
    virtual void set_added_mass_matrix(const Eigen::Matrix<double, 6, 6>& matrix) override;
    virtual Eigen::Matrix<double, 6, 6> get_added_mass_matrix() const override;

    virtual void set_position(const Vector3d& position) override;
    virtual Vector3d get_position() const override;
    virtual void set_velocity(const Vector3d& velocity) override;
    virtual Vector3d get_velocity() const override;
    virtual void set_acceleration(const Vector3d& acceleration) override;
    virtual Vector3d get_acceleration() const override;
    virtual void set_rotational_velocity(const Vector3d& rotational_velocity, bool is_local = true) override;
    virtual Vector3d get_rotational_velocity(bool is_local = true) const override;
    virtual void set_rotational_acceleration(const Vector3d& rotational_acceleration, bool is_local = true) override;
    virtual Vector3d get_rotational_acceleration(bool is_local = true) const override;
};

/**
 * @brief Chrono elasto element class.
 */
class ElementElastoChrono : public virtual ElementElasto {
  public:
    std::shared_ptr<chrono::fea::ChElementBeam> chobj;
    virtual void set_nodes(std::shared_ptr<NodeElasto> node1, std::shared_ptr<NodeElasto> node2) override;
    virtual void evaluate_position_rotation(double eta, Vector3d& position, Quaternion& rotation) override;
    virtual void evaluate_force_torque(double eta, Vector3d& force, Vector3d& torque) override;
    virtual double get_mass() override;
    virtual void update_properties();
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
    virtual void update_properties() override;
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
    virtual void update_properties() override;
};

/**
 * @brief Chrono elasto mooring element class.
 */
class ElementMooringElastoChrono : public ElementElastoChrono, public ElementMooringElasto {
  public:
    /** @brief Pointer to underlying Chrono object. */
    std::shared_ptr<chrono::fea::ChElementCableANCF> chobj;

    ElementMooringElastoChrono();
    virtual void set_nodes(std::shared_ptr<NodeElasto> node1, std::shared_ptr<NodeElasto> node2) override;
    virtual void set_properties(double density,
                                double diameter,
                                double stiffness_axial,
                                double stiffness_bending) override;
    virtual void set_rest_length(double rest_length) override;
    virtual double get_rest_length() const override;
};

class LinkChronoBase {
  public:
    /** @brief Pointer to underlying Chrono object. */
    std::shared_ptr<chrono::ChLinkBase> chobj;
};

class SpringLinearChrono : public SpringLinear {
  public:
    /** @brief Pointer to underlying Chrono object. */
    std::shared_ptr<chrono::ChLinkTSDA> chobj;

    SpringLinearChrono();
    virtual void initialize(const BodyElasto& body1, const BodyElasto& body2) override;
    virtual void initialize_with_anchors(const BodyElasto& body1,
                                         const BodyElasto& body2,
                                         bool local,
                                         const Vector3d& anchor1,
                                         const Vector3d& anchor2) override;
    virtual void set_rest_length(double rest_length) override;
    virtual void set_spring_coefficient(double spring_coefficient) override;
    virtual void set_damping_coefficient(double damping_coefficient) override;
    virtual double get_force() override;
};

/**
 * @brief Chrono link class.
 */
class LinkChrono : public Link, public LinkChronoBase {
  public:
    /** @brief Pointer to underlying Chrono object. */
    std::shared_ptr<chrono::ChLinkMateGeneric> chobj;

    LinkChrono();
    virtual void set_constraints(bool surge, bool sway, bool heave, bool roll, bool pitch, bool yaw) override;

    virtual void initialize(const Entity& entity1, const Entity& entity2) override;
    Vector3d get_reaction_force() const override;
    Vector3d get_reaction_torque() const override;
};

/**
 * @brief Chrono link class for cables.
 */
class LinkChronoCable : public Link, public LinkChronoBase {
  public:
    /** @brief Pointer to underlying Chrono object. */
    std::shared_ptr<chrono::ChLinkBase> chobj;

    LinkChronoCable();
    virtual void set_constraints(bool surge, bool sway, bool heave, bool roll, bool pitch, bool yaw) override;
    virtual void initialize(const Entity& entity1, const Entity& entity2) override;
    Vector3d get_reaction_force() const override;
    Vector3d get_reaction_torque() const override;
};

/**
 * @brief Chrono link class using stiffness and damping matrices.
 */
class LinkMatrixStiffnessDampingChrono : public LinkMatrixStiffnessDamping {
  public:
    /** @brief Pointer to underlying Chrono object. */
    std::shared_ptr<chrono::ChLoadBodyBodyBushingGeneric> chobj;
    Eigen::Matrix<double, 6, 6> stiffness_matrix;
    Eigen::Matrix<double, 6, 6> damping_matrix;

    LinkMatrixStiffnessDampingChrono();
    void initialize(const Entity& entity1, const Entity& entity2) override;
    void set_stiffness_matrix(const Eigen::Matrix<double, 6, 6>& stiffness_matrix) override;
    void set_damping_matrix(const Eigen::Matrix<double, 6, 6>& damping_matrix) override;
};

/**
 * @brief Chrono elasto mesh class.
 */
class MeshElastoChrono : public MeshElasto {
  public:
    /** @brief Pointer to underlying Chrono object. */
    std::shared_ptr<chrono::fea::ChMesh> chobj;
    std::shared_ptr<chrono::ChLoadContainer> chloadcontainer;

    MeshElastoChrono();
    virtual void add(NodeElasto& node) override;
    virtual void add(ElementElasto& element) override;
};

/**
 * @brief Chrono elasto system class.
 */
class SystemElastoChrono : public SystemElasto {
  public:
    /** @brief Pointer to underlying Chrono object. */
    std::shared_ptr<chrono::ChSystem> chobj;
    std::shared_ptr<chrono::ChLoadContainer> chloadcontainer;

    SystemElastoChrono();
    virtual void assemble() override;
    virtual void presetup(double fraction) override;
    virtual void step(double dt) override;
    virtual double get_time() const override;
    virtual void set_time(double time) override;
    virtual void do_statics(bool linear, int nonlinear_steps) override;
    virtual Vector3d get_gravitational_acceleration() const override;
    virtual void set_gravitational_acceleration(const Vector3d& gravitational_acceleration) override;
    virtual void add(BodyElasto& body) override;
    virtual void add(MeshElasto& mesh) override;
    virtual void add(Link& link) override;
    virtual void add(LinkMatrixStiffnessDamping& link) override;
    virtual void add(SpringLinear& spring) override;
};

}  // namespace elasto
}  // namespace seahowl
