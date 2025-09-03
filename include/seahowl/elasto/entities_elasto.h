#pragma once

#include "seahowl/commons/entities.h"

#include <vector>
#include <memory>

namespace seahowl {
///@brief Elastodynamic model module
namespace elasto {

/**
 * @brief Elasto loadable entiry base class.
 */
class EntityLoadable : public virtual EntityDynamic {
  public:
    /**
     * @brief Resets forces and moments of loadable entity.
     */
    virtual void reset_loads() = 0;

    /**
     * @brief Resets internal forces and moments. Do not call manually.
     */
    virtual void reset_loads_internals() = 0;

    /**
     * @brief Returns applied force on entity.
     *
     * @param[in] is_local Whether the force is returned from local or global reference frame.
     */
    virtual Vector3d get_force(bool is_local = false) const = 0;

    /**
     * @brief Returns applied internal force on entity. For internal workflow.
     *
     * @param[in] is_local Whether the force is returned from local or global reference frame.
     */
    virtual Vector3d get_force_internals(bool is_local = false) const = 0;

    /**
     * @brief Returns total applied force (internal and external) on entity.
     *
     * @param[in] is_local Whether the force is returned from local or global reference frame.
     */
    virtual Vector3d get_force_total(bool is_local = false) const;

    /**
     * @brief Returns applied torque on entity.
     *
     * @param[in] is_local Whether the torque is returned from local or global reference frame.
     */
    virtual Vector3d get_torque(bool is_local = true) const = 0;

    /**
     * @brief Returns applied internal torque on entity. For internal workflow.
     *
     * @param[in] is_local Whether the torque is returned from local or global reference frame.
     */
    virtual Vector3d get_torque_internals(bool is_local = true) const = 0;

    /**
     * @brief Returns total applied torque (internal and external) on entity.
     *
     * @param[in] is_local Whether the torque is returned from local or global reference frame.
     */
    virtual Vector3d get_torque_total(bool is_local = true) const;

    /**
     * @brief Sets force on entity.
     *
     * @param[in] force Force to be set.
     * @param[in] is_local Whether the force is applied from local or global reference frame.
     */
    virtual void set_force(const Vector3d& force, bool is_local = false) = 0;

    /**
     * @brief Sets torque on entity.
     *
     * @param[in] torque Torque to be set.
     * @param[in] is_local Whether the torque is applied from local or global reference frame.
     */
    virtual void set_torque(const Vector3d& torque, bool is_local = true) = 0;

    /**
     * @brief Accumulates force on entity.
     *
     * @param[in] force Force to be accumulated.
     * @param[in] is_local Whether the force is applied from local or global reference frame.
     */
    virtual void accumulate_force(const Vector3d& force, bool is_local = false) = 0;

    /**
     * @brief Accumulates internal force on entity. Do not call manually.
     *
     * @param[in] force Force to be accumulated.
     * @param[in] is_local Whether the force is applied from local or global reference frame.
     */
    virtual void accumulate_force_internals(const Vector3d& force, bool is_local = false) = 0;

    /**
     * @brief Accumulates torque on entity.
     *
     * @param[in] torque Torque to be accumulated.
     * @param[in] is_local Whether the torque is applied from local or global reference frame.
     */
    virtual void accumulate_torque(const Vector3d& torque, bool is_local = true) = 0;

    /**
     * @brief Accumulates internal torque on entity. Do not call manually.
     *
     * @param[in] torque Torque to be accumulated.
     * @param[in] is_local Whether the torque is applied from local or global reference frame.
     */
    virtual void accumulate_torque_internals(const Vector3d& torque, bool is_local = true) = 0;

    /**
     * @brief Sets added mass matrix of entity.
     *
     * @param[in] matrix Added mass matrix.
     */
    virtual void set_added_mass_matrix(const Eigen::Matrix<double, 6, 6>& matrix) = 0;

    /**
     * @brief Accumulates (adds to) added mass matrix of entity.
     *
     * @param[in] matrix Added mass matrix to accumulate.
     */
    virtual void accumulate_added_mass_matrix(const Eigen::Matrix<double, 6, 6>& matrix) = 0;

    /**
     * @brief Returns added mass matrix of entity.
     */
    virtual Eigen::Matrix<double, 6, 6> get_added_mass_matrix() const = 0;

    /**
     * @brief Sets damping matrix of entity.
     *
     * @param[in] matrix Damping matrix.
     */
    virtual void set_damping_matrix(const Eigen::Matrix<double, 6, 6>& matrix) = 0;

    /**
     * @brief Accumulates (adds to) damping matrix of entity.
     *
     * @param[in] matrix Damping matrix to accumulate.
     */
    virtual void accumulate_damping_matrix(const Eigen::Matrix<double, 6, 6>& matrix) = 0;

    /**
     * @brief Returns damping matrix of entity.
     */
    virtual Eigen::Matrix<double, 6, 6> get_damping_matrix() const = 0;
};

/**
 * @brief Elasto rigid body base class.
 */
class BodyElasto : public virtual EntityLoadable {
  public:
    /**
     * @brief Sets mass of body.
     *
     * @param[in] mass Mass of body.
     */
    virtual void set_mass(double mass) = 0;

    /**
     * @brief Get mass of body.
     */
    virtual double get_mass() = 0;

    /**
     * @brief Sets inertia (diagonal terms) of body.
     *
     * @param[in] inertia Diagonal inertia to apply.
     */
    virtual void set_inertia_diagonal(const Vector3d& inertia) = 0;

    /**
     * @brief Sets inertia matrix (3x3) of body.
     *
     * @param[in] inertia Inertia matrix of body.
     */
    virtual void set_inertia_matrix(const Eigen::Matrix<double, 3, 3>& inertia) = 0;

    /**
     * @brief Returns inertia matrix (3x3) of body.
     */
    virtual Eigen::Matrix<double, 3, 3> get_inertia_matrix() const = 0;

    /**
     * @brief Fix body in space.
     *
     * param[in] is_fixed Fixed if true, free if false.
     */
    virtual void set_fixed(bool is_fixed) = 0;

    /**
     * @brief Returns whether body is fixed (true) or not (false).
     */
    virtual bool is_fixed() const = 0;
};

/**
 * @brief Elasto node base class.
 */
class NodeElasto : public virtual EntityLoadable {
  public:
    /**
     * @brief Sets mass of node.
     *
     * @param[in] mass Mass of node.
     */
    virtual void set_mass(double mass) = 0;

    /**
     * @brief Get mass of node.
     */
    virtual double get_mass() = 0;

    /**
     * @brief Fix node in space.
     *
     * param[in] is_fixed Fixed if true, free if false.
     */
    virtual void set_fixed(bool is_fixed) = 0;

    /**
     * @brief Returns whether node is fixed (true) or not (false).
     */
    virtual bool is_fixed() const = 0;
};

/**
 * @brief Elasto element base class.
 */
class ElementElasto {
  public:
    /** @brief Nodes of elasto element. */
    std::vector<std::shared_ptr<NodeElasto>> nodes;
    /**
     * @brief Virtual destructor.
     */
    virtual ~ElementElasto() = default;
    /**
     * @brief Sets nodes of element.
     *
     * param[in] node1 First node of element.
     * param[in] node1 Second node of element.
     */
    virtual void set_nodes(std::shared_ptr<NodeElasto> node1, std::shared_ptr<NodeElasto> node2) = 0;

    /**
     * @brief Evaluates position and rotation of point within element.
     *
     * param[in] eta Normalized abscissa along element within range [-1, +1].
     * param[out] position Position to evaluate.
     * param[out] rotation Rotation to evaluate.
     */
    virtual void evaluate_position_rotation(double eta, Vector3d& position, Quaternion& rotation) const = 0;

    /**
     * @brief Evaluates force and torque of point within element.
     *
     * param[in] eta Normalized abscissa along element within range [-1, +1].
     * param[out] force Force to evaluate.
     * param[out] torque Torque to evaluate.
     */
    virtual void evaluate_force_torque(double eta, Vector3d& force, Vector3d& torque) const = 0;

    /**
     * @brief Returns position of point within element.
     *
     * param[in] eta Normalized abscissa along element within range [-1, +1].
     */
    Vector3d get_position(double eta) const;

    /**
     * @brief Returns rotation of point within element.
     *
     * param[in] eta Normalized abscissa along element within range [-1, +1].
     */
    Quaternion get_rotation(double eta) const;

    /**
     * @brief Returns force of point within element.
     *
     * param[in] eta Normalized abscissa along element within range [-1, +1].
     */
    Vector3d get_force(double eta) const;

    /**
     * @brief Returns force of point within element.
     *
     * param[in] eta Normalized abscissa along element within range [-1, +1].
     */
    Vector3d get_torque(double eta) const;

    /**
     * @brief Returns mass of elasto element.
     */
    virtual double get_mass() = 0;
};

/**
 * @brief Elasto blade element base class.
 */
class ElementBladeElasto : public virtual ElementElasto {
  public:
    /**
     * @brief Sets prebend of blade elasto element.
     *
     * @param[in] prebend Prebend to apply.
     */
    virtual void set_prebend(const Quaternion& prebend) = 0;
};

/**
 * @brief Elasto mooring element base class.
 */
class ElementMooringElasto : public virtual ElementElasto {
  public:
    /**
     * @brief Sets properties of mooring elasto element.
     *
     * @param[in] density Density of element.
     * @param[in] diameter Diameter of element.
     * @param[in] stiffness_axial Axial stiffness of element.
     * @param[in] stiffness_bending Bending stiffness of element.
     */
    virtual void set_properties(double density, double diameter, double stiffness_axial, double stiffness_bending) = 0;

    /**
     * @brief Sets rest length of mooring elasto element.
     *
     * @param[in] rest_length Rest length of element.
     */
    virtual void set_rest_length(double rest_length) = 0;

    /**
     * @brief Returns rest length of mooring elasto element.
     */
    virtual double get_rest_length() const = 0;
};

/**
 * @brief Linear spring base class.
 */
class SpringLinear {
  public:
    /**
     * @brief Initialize spring between bodies.
     *
     * @param[in] body1 First body to link.
     * @param[in] body2 Second body to link.
     */
    virtual void initialize(const BodyElasto& body1, const BodyElasto& body2) = 0;
    /**
     * @brief Virtual destructor.
     */
    virtual ~SpringLinear() = default;

    /**
     * @brief Initialize spring between bodies with anchor offsets.
     *
     * @param[in] body1 First body to link.
     * @param[in] body2 Second body to link.
     * @param[in] local Whether anchor locations are in local body frames or global frame.
     * @param[in] anchor1 Position of anchor for body1.
     * @param[in] anchor2 Position of anchor for body2.
     */
    virtual void initialize_with_anchors(const BodyElasto& body1,
                                         const BodyElasto& body2,
                                         bool local,
                                         const Vector3d& anchor1,
                                         const Vector3d& anchor2) = 0;

    /**
     * @brief Sets rest length of spring.
     *
     * @param[in] rest_length Rest length of spring.
     */
    virtual void set_rest_length(double rest_length) = 0;

    /**
     * @brief Sets spring coefficient (K).
     *
     * @param[in] spring_coefficient Spring coefficient (K).
     */
    virtual void set_spring_coefficient(double spring_coefficient) = 0;

    /**
     * @brief Sets damping coefficient (D).
     *
     * @param[in] damping_coefficient Damping coefficient (D).
     */
    virtual void set_damping_coefficient(double damping_coefficient) = 0;

    /**
     * @brief Returns force.
     */
    virtual double get_force() = 0;
};

/**
 * @brief Elasto link/joint base class.
 */
class Link {
  public:
    /**
     * @brief Destructor.
     */
    virtual ~Link() = default;
    /**
     * @brief Initialize link between entitiies.
     *
     * @param[in] entity1 First entity to link.
     * @param[in] entity2 Second entity to link.
     */
    virtual void initialize(const Entity& entity1, const Entity& entity2) = 0;

    /**
     * @brief Sets constraints (true: constrained; false: unconstrained).
     *
     * @param[in] surge Translational x-axis constraint.
     * @param[in] sway Translational y-axis constraint.
     * @param[in] heave Translational z-axis constraint.
     * @param[in] roll Rotational x-axis constraint.
     * @param[in] pitch Rotational y-axis constraint.
     * @param[in] yaw Rotational z-axis constraint.
     */
    virtual void set_constraints(bool surge, bool sway, bool heave, bool roll, bool pitch, bool yaw) = 0;

    /**
     * @brief Returns reaction force.
     */
    virtual Vector3d get_reaction_force() const = 0;

    /**
     * @brief Returns reaction torque.
     */
    virtual Vector3d get_reaction_torque() const = 0;
};

/**
 * @brief Link class using stiffness and damping matrices.
 */
class LinkMatrixStiffnessDamping {
  public:
    virtual ~LinkMatrixStiffnessDamping() = default;
    /**
     * @brief Initialize link between entitiies.
     *
     * @param[in] entity1 First entity to link.
     * @param[in] entity2 Second entity to link.
     */
    virtual void initialize(const Entity& entity1, const Entity& entity2) = 0;

    /**
     * @brief Sets stiffness matrix of link.
     *
     * @param[in] stiffness_matrix Stiffness matrix for link (6x6).
     */
    virtual void set_stiffness_matrix(const Eigen::Matrix<double, 6, 6>& stiffness_matrix) = 0;

    /**
     * @brief Sets damping matrix of link.
     *
     * @param[in] damping_matrix Damping matrix for link (6x6).
     */
    virtual void set_damping_matrix(const Eigen::Matrix<double, 6, 6>& damping_matrix) = 0;
};

/**
 * @brief Actuator class for imposing rotation between two bodies.
 *
 * Rotation happens around the Z-axis of the controller body (body_controller).
 * The axis can be changed by providing reference_rotation to the actuator.
 * Reference rotation is expressed in the local coordinate system of controller body.
 */
class ActuatorRotation : public virtual Entity {
  public:
    /** @brief Body of the controller (manages motor). */
    std::unique_ptr<BodyElasto> body_controller;
    /** @brief Body of the worker (rotating around controller body). */
    std::unique_ptr<BodyElasto> body_worker;
    /** @brief Reference rotation to change axis from local the Z-axis of controller body. */
    Quaternion reference_rotation;
    /** @brief Link for fixing the actuator in space. */
    std::unique_ptr<Link> link;
    /**
     * @brief Destructor.
     */
    virtual ~ActuatorRotation() = default;
    /**
     * @brief Resets actuator (bodies back to origin and control timeseries to zero).
     */
    virtual void reset() = 0;

    /**
     * @brief Sets timeseries for actuator.
     *
     * @param[in] time_array Time array.
     * @param[in] values_array Values array (angles).
     */
    virtual void set_control_timeseries(const std::vector<double>& time_array,
                                        const std::vector<double>& values_array) = 0;

    /**
     * @brief Returns wanted value for actuator at given time.
     *
     * @param[in] time Time value.
     */
    virtual double get_control_value(double time) const = 0;

    /**
     * @brief Imposes a constant value.
     *
     * @param[in] value Value to impose.
     */
    virtual void impose_value_constant(double value) = 0;

    /**
     * @brief Increments value to current angle and make it constant.
     *
     * @param[in] value Value to impose.
     */
    virtual void increment_value_constant(double value) = 0;

    /**
     * @brief Returns current angle between the 2 actuator bodies.
     */
    virtual double get_angle() const = 0;

    /**
     * @brief Returns the rotation axis of the actuator.
     */
    Vector3d get_rotation_axis() const;

    /**
     * @brief Sets whether the actuator if fixed or not (keep it false for actuator dynamics).
     *
     * @param[in] is_fixed Whether the actuator is fixed or not.
     */
    virtual void set_fixed_actuator(bool is_fixed) = 0;

    /**
     * @brief Returns whether the actuator is fixed or not.
     */
    virtual bool is_fixed_actuator() const = 0;

    virtual void initialize_links() = 0;

    void set_position(const Vector3d& position) override;
    Vector3d get_position() const override;
    void set_rotation(const Quaternion& rotation) override;
    Quaternion get_rotation() const override;
    Vector3d get_rpy_angles() const override;
};

/**
 * @brief Elasto mesh base class.
 */
class MeshElasto {
  public:
    /**
     * @brief Virtual destructor.
     */
    virtual ~MeshElasto() = default;
    /**
     * @brief Adds node to mesh.
     *
     * @param[in] node Node to add to mesh.
     */
    virtual void add(NodeElasto& node) = 0;

    /**
     * @brief Adds element to mesh.
     *
     * @param[in] element Element to add to mesh.
     */
    virtual void add(ElementElasto& element) = 0;
};

}  // namespace elasto
}  // namespace seahowl
