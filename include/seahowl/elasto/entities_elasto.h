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
    /**
     * @brief Resets forces and moments of loadable entity.
     */
    virtual void reset_loads() = 0;

    /**
     * @brief Returns applied force on entity.
     *
     * @param[in] is_local Whether the force is returned from local or global reference frame.
     */
    virtual Vector3d get_force(bool is_local = false) const = 0;

    /**
     * @brief Returns applied torque on entity.
     *
     * @param[in] is_local Whether the torque is returned from local or global reference frame.
     */
    virtual Vector3d get_torque(bool is_local = true) const = 0;

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
     * @brief Accumulates torque on entity.
     *
     * @param[in] torque Torque to be accumulated.
     * @param[in] is_local Whether the torque is applied from local or global reference frame.
     */
    virtual void accumulate_torque(const Vector3d& torque, bool is_local = true) = 0;
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
};

/**
 * @brief Elasto node base class.
 */
class NodeElasto : public virtual EntityLoadable {
  public:
    /**
     * @brief Returns main direction of node.
     */
    virtual Vector3d get_direction() const = 0;

    /**
     * @brief Fix node in space.
     *
     * param[in] is_fixed Fixed if true, free if false.
     */
    virtual void set_fixed(bool is_fixed) = 0;
};

/**
 * @brief Elasto element base class.
 */
class ElementElasto {
  public:
    /** @brief Nodes of elasto element. */
    std::vector<std::shared_ptr<NodeElasto>> nodes;

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
    virtual void evaluate_position_rotation(double eta, Vector3d& position, Quaternion& rotation) = 0;

    /**
     * @brief Evaluates force and torque of point within element.
     *
     * param[in] eta Normalized abscissa along element within range [-1, +1].
     * param[out] force Force to evaluate.
     * param[out] torque Torque to evaluate.
     */
    virtual void evaluate_force_torque(double eta, Vector3d& force, Vector3d& torque) = 0;

    /**
     * @brief Returns force of point within element.
     *
     * param[in] eta Normalized abscissa along element within range [-1, +1].
     */
    Vector3d get_force(double eta) {
        auto force = Vector3d(0.0, 0.0, 0.0);
        auto torque = Vector3d(0.0, 0.0, 0.0);
        evaluate_force_torque(eta, force, torque);
        return force;
    };

    /**
     * @brief Returns force of point within element.
     *
     * param[in] eta Normalized abscissa along element within range [-1, +1].
     */
    Vector3d get_torque(double eta) {
        auto force = Vector3d(0.0, 0.0, 0.0);
        auto torque = Vector3d(0.0, 0.0, 0.0);
        evaluate_force_torque(eta, force, torque);
        return torque;
    };

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
 * @brief Elasto link/joint base class.
 */
class Link {
  public:
    /**
     * @brief Initialize link between bodies.
     */
    virtual void initialize(const BodyElasto& body1, const BodyElasto& body2) = 0;

    /**
     * @brief Initialize link between node and body.
     */
    virtual void initialize(const NodeElasto& node1, const BodyElasto& body2) = 0;

    /**
     * @brief Initialize link between body and node.
     */
    void initialize(const BodyElasto& body1, const NodeElasto& node2) { initialize(node2, body1); };

    /**
     * @brief Initialize link between node and node.
     */
    virtual void initialize(const NodeElasto& node1, const NodeElasto& node2) = 0;

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
 * @brief Elasto mesh base class.
 */
class MeshElasto {
  public:
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
