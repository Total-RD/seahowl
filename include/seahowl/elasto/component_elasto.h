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
    virtual void build() = 0;
    /**
     * @brief Virtual destructor.
     */
    virtual ~ComponentElasto() = default;

    /**
     * @brief Assembles the component.
     */
    void assemble(SystemElasto& system);

    /**
     * @brief Initializes component.
     */
    virtual void initialize(){};

    /**
     * @brief Presetup of component.
     *
     * @param[in] fraction Fraction of presetup phase, starting at 0.0 and ending at 1.0.
     */
    virtual void presetup(double fraction){};

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

  protected:
    bool is_assembled = false;
    virtual void assemble_this(SystemElasto& system) = 0;
};

/**
 * @brief Elasto component with discretization support.
 *
 * Intermediate class for elasto components that have discretization fractions.
 */
class ComponentElastoDiscretized : public virtual ComponentElasto {
  public:
    /** @brief Discretization fractions (normalized abscissa) in the range [0, 1]. */
    std::vector<double> discretization_fractions{};
};

/**
 * @brief Finite Element Elasto component base class.
 *
 * All FEA elasto component classes are derived from this class.
 */
class ComponentElastoFEA : public virtual ComponentElastoDiscretized {
  public:
    /** @brief Finite element nodes. */
    std::vector<std::shared_ptr<NodeElasto>> nodes;
    /** @brief Finite element beams. */
    std::vector<std::shared_ptr<ElementElasto>> elements;

    /**
     * @brief Builds nodes for the elasto component based on reference points.
     *
     * @param[in] discretized_points Reference points from which the FEA nodes are built.
     */
    void build_nodes(const std::vector<ReferencePointElasto>& discretized_points);

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
     * @brief Evaluates the position and rotation (interpolated with slerp) given the abscissa of an element.
     *
     * @param[out] position Position vector to use for evaluation (will be overwritten).
     * @param[out] rotation Rotation quaternion to use for evaluation (will be overwritten).
     * @param[in] element_index Index of the element on which the evaluation is undertaken.
     * @param[in] eta Abscissa of the element within the range [-1, +1], with -1 at node1 and +1 at node2.
     */
    virtual void evaluate_position_rotation_slerp(Vector3d& position,
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
     * @brief Accumulates added mass matrix on a given FEA element.
     *
     * @param[in] matrix Added mass matrix to accumulate.
     * @param[in] element_index Index of the element on which the load is accumulated.
     * @param[in] eta Abscissa of the element within the range [-1, +1], with -1 at node1 and +1 at node2.
     * @param[in] offset Offset from given abscissa along longitudinal axis of element.
     */
    void accumulate_mass_matrix(const Eigen::Matrix<double, 6, 6>& matrix, int element_index, double eta);

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

    /**
     * @brief Returns entity along component.
     *
     * @param[in] eta Normalized abscissa between node1 and node2 of element.
     * @param[in] element Element index.
     */
    seahowl::EntityDynamicEigen get_entity_along_component(double eta, int element_index) const;

    /**
     * @brief Returns entity along component (with rotation interpolated using slerp).
     *
     * @param[in] eta Normalized abscissa between node1 and node2 of element.
     * @param[in] element Element index.
     */
    seahowl::EntityDynamicEigen get_entity_along_component_slerp(double eta, int element_index) const;

  protected:
    /**
     * @brief Assembles the FEA component (adds all nodes and elements to mesh).
     */
    virtual void assemble_this(SystemElasto& system) override;
};

}  // namespace elasto
}  // namespace seahowl
