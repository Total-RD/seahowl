#pragma once

#include "seahowl/commons/numerics.h"
#include "seahowl/elasto/entities_elasto.h"

#include <vector>
#include <memory>

// forward declarations
namespace seahowl {
namespace elasto {
class SystemElasto;
struct ReferencePointElasto;
}  // namespace elasto
}  // namespace seahowl

namespace seahowl {
///@brief Elastodynamic model module
namespace elasto {

/**
 * @brief Elasto component base class.
 *
 * All elasto component classes are derived from this class.
 */
class ComponentElasto {
  public:
    /**
     * @brief Builds the component (to call before assemble).
     */
    virtual void build(){};

    /**
     * @brief Assembles the component.
     *
     * @param[out] mesh System on which to assemble component.
     */
    virtual void assemble(SystemElasto& system){};

    /**
     * @brief Initializes component.
     */
    virtual void initialize(){};

    /**
     * @brief Resets accumulated loads of component.
     */
    virtual void reset_loads(){};

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
class ComponentElastoFEA : public virtual ComponentElasto {
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
     * @param[out] mesh System on which to add nodes and elements.
     */
    virtual void assemble(SystemElasto& system) override;

    virtual void rotate(double angle, const Vector3d& axis) const override;
    virtual void translate(const Vector3d& translation_vector) const override;
    virtual double get_mass() const override;

    /**
     * @brief Resets accumulated loads at nodes of FEA component.
     */
    virtual void reset_loads() override;

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
     * @param[in] load Moment vector to accumulate.
     * @param[in] element_index Index of the element on which the load is accumulated.
     * @param[in] eta Abscissa of the element within the range [-1, +1], with -1 at node1 and +1 at node2.
     * @param[in] offset Offset from given abscissa along longitudinal axis of element.
     */
    void accumulate_element_load(const Vector3d& load,
                                 const Vector3d& moment,
                                 int element_index,
                                 double eta,
                                 const Vector3d& offset);

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
