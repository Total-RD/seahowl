#include <seahowl/elasto/elasto.h>

#include <vector>
#include <numeric>

using namespace seahowl::elasto;
using namespace seahowl;

void ComponentElastoFEA::build_nodes(const std::vector<ReferencePointElasto>& discretized_points) {
    nodes.clear();
    const auto nnodes = discretized_points.size();

    for (size_t ii = 0; ii < nnodes; ii++) {
        auto& discretized_point = discretized_points[ii];
        auto& node_pos = discretized_point.coordinates;

        // get node coordinate system
        Vector3d node_axis;
        chrono::ChMatrix33<> node_rotation;
        if (ii == 0) {
            node_axis = (discretized_points[ii + 1].coordinates - node_pos).normalized();
            node_rotation.Set_A_Xdir(node_axis, chrono::VECT_Y);
        } else if (ii == nnodes - 1) {
            node_axis = (node_pos - discretized_points[ii - 1].coordinates).normalized();
            node_rotation.Set_A_Xdir(node_axis, chrono::VECT_Y);
        } else {
            node_axis = (discretized_points[ii + 1].coordinates - discretized_points[ii - 1].coordinates).normalized();
            node_rotation.Set_A_Xdir(node_axis, chrono::VECT_Y);
        }

        auto node = std::make_shared<NodeFEA>(node_pos, node_rotation.Get_A_quaternion());
        nodes.push_back(node);
    };
};

void ComponentElastoFEA::assemble(std::shared_ptr<MeshElasto> mesh) const {
    for (auto node : nodes) {
        mesh->add(node);
    }
    for (auto element : elements) {
        mesh->add(element);
    }
}

void ComponentElastoFEA::rotate(double angle, const Vector3d& axis) const {
    auto rotation = Quaternion(Q_from_AngAxis(angle, axis));
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
        [](double total, decltype(elements)::value_type pElem) { return total += pElem->GetMass(); });
}

void ComponentElastoFEA::reset_loads() {
    for (auto& node : nodes) {
        node->SetForce({0.0, 0.0, 0.0});
        node->SetTorque({0.0, 0.0, 0.0});
    }
}

void ComponentElastoFEA::evaluate_position_rotation(Vector3d& position,
                                                    Quaternion& rotation,
                                                    int element_index,
                                                    double eta) const {
    auto& element = elements[element_index];

    element->EvaluateSectionFrame(eta, position, rotation);
}

void ComponentElastoFEA::accumulate_element_load(const Vector3d& load,
                                                 int element_index,
                                                 double eta,
                                                 const Vector3d& offset) {
    // sanity check
    if (element_index >= elements.size() || element_index < 0) {
        throw std::runtime_error("Element index " + std::to_string(element_index) + " does not exist (max " +
                                 std::to_string(elements.size()) + ").");
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
    auto node0 = std::dynamic_pointer_cast<chrono::fea::ChNodeFEAxyzrot>(element->GetNodeN(0));
    node0->SetForce(node0->GetForce() + load0);
    node0->SetTorque(node0->GetTorque() + (position + offset - node0->GetPos()) % load0);
    // load on second node
    double weight1 = 0.5 * abs(eta + 1);
    auto load1 = load * weight1;
    auto node1 = std::dynamic_pointer_cast<chrono::fea::ChNodeFEAxyzrot>(element->GetNodeN(1));
    node1->SetForce(node1->GetForce() + load1);
    node1->SetTorque(node1->GetTorque() + (position + offset - node1->GetPos()) % load1);
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
        rotational_velocities.push_back(node->get_rotational_velocity_local());
    }
    return rotational_velocities;
}

std::vector<Vector3d> ComponentElastoFEA::get_nodes_rotational_accelerations() const {
    std::vector<Vector3d> rotational_accelerations;
    for (auto& node : nodes) {
        rotational_accelerations.push_back(node->get_rotational_acceleration_local());
    }
    return rotational_accelerations;
}

std::vector<Vector3d> ComponentElastoFEA::get_nodes_loads() const {
    std::vector<Vector3d> loads;
    for (auto& node : nodes) {
        loads.push_back(node->get_load());
    }
    return loads;
}
