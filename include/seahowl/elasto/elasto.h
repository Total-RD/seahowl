#pragma once

#include <vector>
#include <memory>

#include <seahowl/commons.h>
#include <seahowl/elasto/reference_point_elasto.h>

namespace seahowl {
///@brief Elastodynamic model module
namespace elasto {

/**
 * @brief Elasto rigid body base class.
 */
class BodyElasto : public EntityDynamic {
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
};

/**
 * @brief Elasto node base class.
 */
class NodeElasto : public EntityDynamic {
  public:
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

/**
 * @brief Elasto system base class.
 */
class SystemElasto {
  public:
    /**
     * @brief Returns gravitational acceleration.
     */
    virtual Vector3d get_gravitational_acceleration() const = 0;

    /**
     * @brief Sets gravitational acceleration.
     *
     * @param[in] gravitational_acceleration Gravitational acceleration.
     */
    virtual void set_gravitational_acceleration(Vector3d gravitational_acceleration) = 0;

    /**
     * @brief Adds body to system.
     *
     * @param[in] body Body to add to system.
     */
    virtual void add(std::shared_ptr<BodyElasto> body) = 0;

    /**
     * @brief Adds mesh to system.
     *
     * @param[in] mesh Mesh to add to system.
     */
    virtual void add(std::shared_ptr<MeshElasto> mesh) = 0;

    /**
     * @brief Adds link to system.
     *
     * @param[in] link Link to add to system.
     */
    virtual void add(std::shared_ptr<Link> link) = 0;
};

/**
 * @brief Elasto component base class.
 *
 * All elasto component classes are derived from this class.
 */
class ComponentElasto {
  public:
    /**
     * @brief Rotates the component.
     *
     * @param[in] translation_vector The angle of rotation (in radians).
     * @param[in] axis The axis of rotation (3D vector).
     */
    virtual void rotate(double angle, const Vector3d& axis) const = 0;

    /**
     * @brief Translates the component.
     *
     * @param[in] translation_vector The 3D translation vector.
     */
    virtual void translate(const Vector3d& translation_vector) const = 0;

    /**
     * @brief Returns the mass of the component.
     */
    virtual double get_mass() const = 0;
};

/**
 * @brief Finite Element Elasto component base class.
 *
 * All FEA elasto component classes are derived from this class.
 */
class ComponentElastoFEA : public ComponentElasto {
  public:
    /** @brief Finite element nodes. */
    std::vector<std::shared_ptr<NodeElasto>> nodes;
    /** @brief Finite element beams. */
    std::vector<std::shared_ptr<ElementElasto>> elements;
    /** @brief Discretization fractions (normalized abscissa) in the range [0, 1] to discretize the FEA component. */
    std::vector<double> discretization_fractions;

    /**
     * @brief Builds nodes for the elasto component based on reference points.
     *
     * @param[in] discretized_points Reference points from which the FEA nodes are built.
     */
    void build_nodes(const std::vector<ReferencePointElasto>& discretized_points);

    /**
     * @brief Assembles the FEA component (adds all nodes and elements to mesh).
     *
     * @param[out] mesh Mesh on which to add nodes and elements.
     */
    void assemble(std::shared_ptr<MeshElasto> mesh) const;

    virtual void rotate(double angle, const Vector3d& axis) const override;
    virtual void translate(const Vector3d& translation_vector) const override;
    virtual double get_mass() const override;

    /**
     * @brief Resets accumulated loads at nodes of FEA component.
     */
    void reset_loads();

    /**
     * @brief Evaluates the position and rotation given the abscissa of an element.
     *
     * @param[out] position Position vector to use for evaluation (will be overwritten).
     * @param[out] rotation Rotation quaternion to use for evaluation (will be overwritten).
     * @param[in] element_index Index of the element on which the evaluation is undertaken.
     * @param[in] eta Abscissa of the element within the range [-1, +1], with -1 at node1 and +1 at node2.
     */
    virtual void evaluate_position_rotation(Vector3d& position,
                                            Quaternion& rotation,
                                            int element_index,
                                            double eta) const;

    /**
     * @brief Accumulates load on a given FEA element.
     *
     * @param[in] load Load vector to accumulate.
     * @param[in] element_index Index of the element on which the load is accumulated.
     * @param[in] eta Abscissa of the element within the range [-1, +1], with -1 at node1 and +1 at node2.
     * @param[in] offset Offset from given abscissa along longitudinal axis of element.
     */
    void accumulate_element_load(const Vector3d& load, int element_index, double eta, const Vector3d& offset);

    /**
     * @brief Returns all nodes positions (global frame of reference).
     */
    std::vector<Vector3d> get_nodes_positions() const;

    /**
     * @brief Returns all nodes velocities (global frame of reference).
     */
    std::vector<Vector3d> get_nodes_velocities() const;

    /**
     * @brief Returns all nodes accelerations (global frame of reference).
     */
    std::vector<Vector3d> get_nodes_accelerations() const;

    /**
     * @brief Returns all nodes rotations (global frame of reference).
     */
    std::vector<Quaternion> get_nodes_rotations() const;

    /**
     * @brief Returns all nodes directions (global frame of reference).
     */
    std::vector<Vector3d> get_nodes_directions() const;

    /**
     * @brief Returns all nodes rotational velocities (local frame of reference).
     */
    std::vector<Vector3d> get_nodes_rotational_velocities() const;

    /**
     * @brief Returns all nodes rotational accelerations (local frame of reference).
     */
    std::vector<Vector3d> get_nodes_rotational_accelerations() const;

    /**
     * @brief Returns all nodes loads (global frame of reference).
     */
    std::vector<Vector3d> get_nodes_loads() const;
};

}  // namespace elasto
}  // namespace seahowl
