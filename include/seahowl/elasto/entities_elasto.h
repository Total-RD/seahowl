#pragma once

#include <vector>
#include <memory>

#include <seahowl/commons/entities.h>
#include <seahowl/elasto/reference_point_elasto.h>

namespace seahowl {
///@brief Elastodynamic model module
namespace elasto {

/**
 * @brief Elasto rigid body base class.
 */
class BodyElasto : public virtual EntityDynamic {
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
    virtual void set_inertia_diagonal(Vector3d inertia) = 0;

    /**
     * @brief Resets forces of body.
     */
    virtual void reset_forces() = 0;

    /**
     * @brief Accumulates torque on body.
     *
     * @param[in] torque Torque to be accumulated.
     * @param[in] is_local Whether the torque is applied from local or global reference frame.
     */
    virtual void accumulate_torque(Vector3d torque, bool is_local) = 0;

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
class NodeElasto : public virtual EntityDynamic {
  public:
    /**
     * @brief Returns main direction of node.
     */
    virtual Vector3d get_direction() const = 0;
    /**
     * @brief Sets load on node.
     *
     * @param[in] force Load to apply on node.
     */
    virtual void set_load(Vector3d force) = 0;

    /**
     * @brief Returns load applied on node.
     */
    virtual Vector3d get_load() const = 0;

    /**
     * @brief Sets torque on node.
     */
    virtual void set_torque(Vector3d torque) = 0;

    /**
     * @brief Returns torque applied on node.
     */
    virtual Vector3d get_torque() const = 0;

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
    std::vector<std::shared_ptr<NodeElasto>> nodes0;

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
     * @brief Returns mass of elasto element.
     */
    virtual double get_mass() = 0;
};

/**
 * @brief Elasto blade element base class.
 */
class ElementBladeElasto : public ElementElasto {
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
class ElementMooringElasto : public ElementElasto {
  public:
    /**
     * @brief Sets properties of mooring elasto element.
     *
     * @param[in] density Density of element.
     * @param[in] diameter Diameter of element.
     * @param[in] stiffness_axial Axial stiffness of element.
     */
    virtual void set_properties(double density, double diameter, double stiffness_axial) = 0;
};

/**
 * @brief Elasto link/joint base class.
 */
class Link {
  public:
    /**
     * @brief Initialize link between bodies.
     */
    virtual void initialize(std::shared_ptr<BodyElasto> body1, std::shared_ptr<BodyElasto> body2) = 0;

    /**
     * @brief Initialize link between node and body.
     */
    virtual void initialize(std::shared_ptr<NodeElasto> node1, std::shared_ptr<BodyElasto> body2) = 0;

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
    virtual void add(std::shared_ptr<NodeElasto> node) = 0;

    /**
     * @brief Adds element to mesh.
     *
     * @param[in] element Element to add to mesh.
     */
    virtual void add(std::shared_ptr<ElementElasto> element) = 0;
};

}  // namespace elasto
}  // namespace seahowl
