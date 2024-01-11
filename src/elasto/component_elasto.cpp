#include "seahowl/elasto/component_elasto.h"

#include "seahowl/elasto/reference_point_elasto.h"
#include "seahowl/elasto/system_elasto.h"
#include "seahowl/elasto/chrono_adapters.h"

#include <vector>
#include <numeric>
#include <spdlog/spdlog.h>

using namespace seahowl::elasto;
using namespace seahowl;

void ComponentElastoFEA::build_nodes(const std::vector<ReferencePointElasto>& discretized_points) {
    nodes.clear();
    const auto nnodes = discretized_points.size();

    for (size_t ii = 0; ii < nnodes; ii++) {
        auto& discretized_point = discretized_points[ii];
        auto& node_pos = discretized_point.coordinates;

        // get node main axis (direction)
        Vector3d node_axis;
        if (ii == 0) {
            node_axis = (discretized_points[ii + 1].coordinates - node_pos).normalized();
        } else if (ii == nnodes - 1) {
            node_axis = (node_pos - discretized_points[ii - 1].coordinates).normalized();
        } else {
            node_axis = (discretized_points[ii + 1].coordinates - discretized_points[ii - 1].coordinates).normalized();
        }

        // make coordinate system of node
        // uses IEC convention
        auto zaxis = node_axis;
        auto up = Vector3d(0.0, 1.0, 0.0);
        auto xaxis = up.cross(zaxis).normalized();
        auto yaxis = zaxis.cross(xaxis).normalized();
        Eigen::Matrix3d coordsys;
        coordsys << xaxis.x(), yaxis.x(), zaxis.x(), xaxis.y(), yaxis.y(), zaxis.y(), xaxis.z(), yaxis.z(), zaxis.z();

        // make node
        auto node = std::make_shared<NodeElastoChrono>(node_pos, Quaternion(coordsys).normalized());
        nodes.push_back(node);
    };
};

void ComponentElastoFEA::assemble(SystemElasto& system) {
    for (auto node : nodes) {
        system.mesh->add(*(node.get()));
    }
    for (auto element : elements) {
        system.mesh->add(*(element.get()));
    }
}

void ComponentElastoFEA::rotate(double angle, const Vector3d& axis) const {
    auto rotation = AngleAxisd(angle, axis);
    for (auto& node : nodes) {
        auto new_position = rotation * node->get_position();
        node->set_position(new_position);
        auto new_rotation = (rotation * node->get_rotation()).normalized();
        node->set_rotation(new_rotation);
    }
}

void ComponentElastoFEA::translate(const Vector3d& translation_vector) const {
    for (auto& node : nodes) {
        node->set_position(node->get_position() + translation_vector);
    }
}
double ComponentElastoFEA::get_mass() const {
    return std::accumulate(
        cbegin(elements), cend(elements), 0.0,
        [](double total, decltype(elements)::value_type pElem) { return total += pElem->get_mass(); });
}

void ComponentElastoFEA::reset_loads() {
    for (auto& node : nodes) {
        node->reset_loads();
    }
}

void ComponentElastoFEA::evaluate_position_rotation(Vector3d& position,
                                                    Quaternion& rotation,
                                                    int element_index,
                                                    double eta) const {
    auto& element = elements[element_index];

    element->evaluate_position_rotation(eta, position, rotation);
}

void ComponentElastoFEA::accumulate_element_load(const Vector3d& load,
                                                 const Vector3d& moment,
                                                 int element_index,
                                                 double eta,
                                                 const Vector3d& offset) {
    // sanity check
    if (element_index >= elements.size() || element_index < 0) {
        throw std::runtime_error("Element index " + std::to_string(element_index) +
                                 " does not exist (number of elements: " + std::to_string(elements.size()) + ").");
    }

    // get position and rotation
    Vector3d position{0.0, 0.0, 0.0};
    Quaternion rotation{0.0, 0.0, 0.0, 0.0};
    evaluate_position_rotation(position, rotation, element_index, eta);

    // apply loads
    auto& element = elements[element_index];
    // load on first node
    double weight0 = 0.5 * abs(eta - 1);
    auto load0 = load * weight0;
    auto moment0 = moment * weight0;
    auto node0 = element->nodes[0];
    // load in global reference
    node0->accumulate_force(load0, false);
    node0->accumulate_torque(moment0 + (position + offset - node0->get_position()).cross(load0), false);
    // load on second node
    double weight1 = 0.5 * abs(eta + 1);
    auto load1 = load * weight1;
    auto moment1 = moment * weight1;
    auto node1 = element->nodes[1];
    // load in global reference
    node1->accumulate_force(load1, false);
    node1->accumulate_torque(moment1 + (position + offset - node1->get_position()).cross(load1), false);
}

std::vector<Vector3d> ComponentElastoFEA::get_nodes_positions() const {
    std::vector<Vector3d> positions;
    for (auto& node : nodes) {
        positions.push_back(node->get_position());
    }
    return positions;
}

std::vector<Vector3d> ComponentElastoFEA::get_nodes_velocities() const {
    std::vector<Vector3d> velocities;
    for (auto& node : nodes) {
        velocities.push_back(node->get_velocity());
    }
    return velocities;
}

std::vector<Vector3d> ComponentElastoFEA::get_nodes_accelerations() const {
    std::vector<Vector3d> accelerations;
    for (auto& node : nodes) {
        accelerations.push_back(node->get_acceleration());
    }
    return accelerations;
}

std::vector<Quaternion> ComponentElastoFEA::get_nodes_rotations() const {
    std::vector<Quaternion> rotations;
    for (auto& node : nodes) {
        rotations.push_back(node->get_rotation());
    }
    return rotations;
}

std::vector<Vector3d> ComponentElastoFEA::get_nodes_directions() const {
    std::vector<Vector3d> directions;
    for (auto& node : nodes) {
        directions.push_back(node->get_direction());
    }
    return directions;
}

std::vector<Vector3d> ComponentElastoFEA::get_nodes_rotational_velocities() const {
    std::vector<Vector3d> rotational_velocities;
    for (auto& node : nodes) {
        rotational_velocities.push_back(node->get_rotational_velocity());
    }
    return rotational_velocities;
}

std::vector<Vector3d> ComponentElastoFEA::get_nodes_rotational_accelerations() const {
    std::vector<Vector3d> rotational_accelerations;
    for (auto& node : nodes) {
        rotational_accelerations.push_back(node->get_rotational_acceleration());
    }
    return rotational_accelerations;
}

std::vector<Vector3d> ComponentElastoFEA::get_nodes_loads() const {
    std::vector<Vector3d> loads;
    for (auto& node : nodes) {
        loads.push_back(node->get_force());
    }
    return loads;
}

seahowl::EntityDynamicEigen ComponentElastoFEA::get_entity_along_component(double eta, int element_index) const {
    auto entity = seahowl::EntityDynamicEigen();
    Vector3d new_position;
    Quaternion new_rotation;
    evaluate_position_rotation(new_position, new_rotation, element_index, eta);
    entity.set_position(new_position);
    entity.set_rotation(new_rotation);

    // update properties of aero nodes
    double weight1 = 0.5 * fabs(eta - 1.0);
    double weight2 = 0.5 * fabs(eta + 1.0);
    auto& element = elements[element_index];
    auto& node1 = element->nodes[0];
    auto& node2 = element->nodes[1];
    entity.set_velocity(weight1 * node1->get_velocity() + weight2 * node2->get_velocity());
    entity.set_rotational_velocity(weight1 * node1->get_rotational_velocity() +
                                   weight2 * node2->get_rotational_velocity());
    entity.set_acceleration(weight1 * node1->get_acceleration() + weight2 * node2->get_acceleration());
    entity.set_rotational_acceleration(weight1 * node1->get_rotational_acceleration() +
                                       weight2 * node2->get_rotational_acceleration());

    return entity;
}
