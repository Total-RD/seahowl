#pragma once

#include <vector>
#include <memory>

#include <chrono/core/ChVector.h>
#include <chrono/fea/ChMesh.h>

namespace chrono {
namespace fea {
class ChNodeFEAxyzrot;
class ChElementBeamTaperedTimoshenko;
class ChElementBeam;
}  // namespace fea
}  // namespace chrono

#include <seahowl/elasto/reference_point_elasto.h>
#include <seahowl/elasto/utils_elasto.h>

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
     * @brief Rotates the component.
     *
     * @param[in] translation_vector The angle of rotation (in radians).
     * @param[in] axis The axis of rotation (3D vector).
     */
    virtual void rotate(double angle, const chrono::ChVector<double>& axis) const = 0;

    /**
     * @brief Translates the component.
     *
     * @param[in] translation_vector The 3D translation vector.
     */
    virtual void translate(const chrono::ChVector<double>& translation_vector) const = 0;

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
    std::vector<std::shared_ptr<chrono::fea::ChNodeFEAxyzrot>> nodes;
    /** @brief Finite element beams. */
    std::vector<std::shared_ptr<chrono::fea::ChElementBeam>> elements;
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
    void assemble(std::shared_ptr<chrono::fea::ChMesh> mesh) const;

    virtual void rotate(double angle, const chrono::ChVector<double>& axis) const override;
    virtual void translate(const chrono::ChVector<double>& translation_vector) const override;
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
    virtual void evaluate_position_rotation(chrono::ChVector<double>& position,
                                            chrono::ChQuaternion<double>& rotation,
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
    void accumulate_element_load(const chrono::ChVector<double>& load,
                                 int element_index,
                                 double eta,
                                 const chrono::ChVector<double>& offset);

    /**
     * @brief Returns all nodes positions (global frame of reference).
     */
    std::vector<chrono::ChVector<double>> get_nodes_positions() const;

    /**
     * @brief Returns all nodes velocities (global frame of reference).
     */
    std::vector<chrono::ChVector<double>> get_nodes_velocities() const;

    /**
     * @brief Returns all nodes accelerations (global frame of reference).
     */
    std::vector<chrono::ChVector<double>> get_nodes_accelerations() const;

    /**
     * @brief Returns all nodes rotations (global frame of reference).
     */
    std::vector<chrono::ChQuaternion<double>> get_nodes_rotations() const;

    /**
     * @brief Returns all nodes directions (global frame of reference).
     */
    std::vector<chrono::ChVector<double>> get_nodes_directions() const;

    /**
     * @brief Returns all nodes rotational velocities (local frame of reference).
     */
    std::vector<chrono::ChVector<double>> get_nodes_rotational_velocities() const;

    /**
     * @brief Returns all nodes rotational accelerations (local frame of reference).
     */
    std::vector<chrono::ChVector<double>> get_nodes_rotational_accelerations() const;

    /**
     * @brief Returns all nodes loads (global frame of reference).
     */
    std::vector<chrono::ChVector<double>> get_nodes_loads() const;
};

}  // namespace elasto
}  // namespace seahowl
