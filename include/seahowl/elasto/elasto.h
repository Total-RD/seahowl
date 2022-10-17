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

/**@brief Component "interface"


*/
class ComponentElasto {
  public:
    ///@{
    virtual void rotate(double angle, chrono::ChVector<double> axis) const = 0;     ///< Rotate the system.
    virtual void translate(chrono::ChVector<double> translation_vector) const = 0;  ///< Translate the system.
    virtual double get_mass() const = 0;                                            ///< Get total mass.
    ///@}
};

/**@brief Finite Element Elastodynamic component */
class ComponentElastoFEA : public ComponentElasto {
  public:
    std::vector<std::shared_ptr<chrono::fea::ChNodeFEAxyzrot>> nodes;   ///< Finite element nodes.
    std::vector<std::shared_ptr<chrono::fea::ChElementBeam>> elements;  ///< Beam elements.
    std::vector<double> discretization_fractions;                       ///< Fractions (normalized abscissa).

    ///@{
    void build_nodes(std::vector<ReferencePointElasto>& discretized_points);
    void assemble(std::shared_ptr<chrono::fea::ChMesh> mesh) const;
    virtual void rotate(double angle, chrono::ChVector<double> axis) const override;     ///< Rotate the system.
    virtual void translate(chrono::ChVector<double> translation_vector) const override;  ///< Translate the system.
    virtual double get_mass() const override;                                            ///< Get total mass.
    void reset_loads();
    virtual void evaluate_position_rotation(chrono::ChVector<double>& position,
                                            chrono::ChQuaternion<double>& rotation,
                                            int element_index,
                                            double eta) const;
    void accumulate_element_load(chrono::ChVector<double> load, int element_index, double eta);

    std::vector<chrono::ChVector<double>> get_nodes_positions() const;      ///< Get all nodes positions.
    std::vector<chrono::ChVector<double>> get_nodes_velocities() const;     ///< Get all nodes velocities.
    std::vector<chrono::ChVector<double>> get_nodes_accelerations() const;  ///< Get all nodes accelerations.
    std::vector<chrono::ChQuaternion<double>> get_nodes_rotations() const;  ///< Get all nodes rotations.
    std::vector<chrono::ChVector<double>> get_nodes_directions() const;     ///< Get all nodes directions.
    std::vector<chrono::ChVector<double>> get_nodes_rotational_velocities()
        const;  ///< Get all nodes rotational velocity (local).
    std::vector<chrono::ChVector<double>> get_nodes_rotational_accelerations()
        const;                                                      ///< Get all nodes rotational acceleration (local).
    std::vector<chrono::ChVector<double>> get_nodes_loads() const;  ///< Get all nodes loads.

    ///@}
};

}  // namespace elasto
}  // namespace seahowl
