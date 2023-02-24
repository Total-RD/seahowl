#include <seahowl/elasto/elasto.h>

#include <vector>
#include <numeric>

#include <chrono/fea/ChElementBeamTaperedTimoshenko.h>
#include <chrono/fea/ChMesh.h>

using namespace seahowl::elasto;

void ComponentElastoFEA::build_nodes(const std::vector<ReferencePointElasto>& discretized_points) {
    nodes.clear();
    const auto nnodes = discretized_points.size();

    for (size_t ii = 0; ii < nnodes; ii++) {
        auto& discretized_point = discretized_points[ii];
        auto& node_pos = discretized_point.coordinates;

        // get node coordinate system
        chrono::ChVector<> node_axis;
        chrono::ChMatrix33<> node_rotation;
        if (ii == 0) {
            node_axis = (discretized_points[ii + 1].coordinates - node_pos).GetNormalized();
            node_rotation.Set_A_Xdir(node_axis, chrono::VECT_Y);
        } else if (ii == nnodes - 1) {
            node_axis = (node_pos - discretized_points[ii - 1].coordinates).GetNormalized();
            node_rotation.Set_A_Xdir(node_axis, chrono::VECT_Y);
        } else {
            node_axis =
                (discretized_points[ii + 1].coordinates - discretized_points[ii - 1].coordinates).GetNormalized();
            node_rotation.Set_A_Xdir(node_axis, chrono::VECT_Y);
        }
        auto node_frame = chrono::ChFrame<>(node_pos, node_rotation);

        // make node
        auto node = chrono_types::make_shared<chrono::fea::ChNodeFEAxyzrot>(node_frame);
        // add node to blade nodes vector
        nodes.push_back(node);
    };
};

void ComponentElastoFEA::assemble(std::shared_ptr<chrono::fea::ChMesh> mesh) const {
    for (auto node : nodes) {
        mesh->AddNode(node);
    }
    for (auto element : elements) {
        mesh->AddElement(element);
    }
}

void ComponentElastoFEA::rotate(double angle, const chrono::ChVector<double>& axis) const {
    auto rotation = Q_from_AngAxis(angle, axis);
    for (auto& node : nodes) {
        auto new_position = rotation.Rotate(node->GetPos());
        node->SetPos(new_position);
        auto new_rotation = (rotation * node->GetRot()).GetNormalized();
        node->SetRot(new_rotation);
    }
}

void ComponentElastoFEA::translate(const chrono::ChVector<double>& translation_vector) const {
    for (auto& node : nodes) {
        node->SetPos(node->GetPos() + translation_vector);
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

void ComponentElastoFEA::evaluate_position_rotation(chrono::ChVector<double>& position,
                                                    chrono::ChQuaternion<double>& rotation,
                                                    int element_index,
                                                    double eta) const {
    auto& element = elements[element_index];

    element->EvaluateSectionFrame(eta, position, rotation);
}

void ComponentElastoFEA::accumulate_element_load(const chrono::ChVector<double>& load,
                                                 int element_index,
                                                 double eta,
                                                 const chrono::ChVector<double>& offset) {
    // sanity check
    if (element_index >= elements.size() || element_index < 0) {
        throw std::runtime_error("Element index " + std::to_string(element_index) + " does not exist (max " +
                                 std::to_string(elements.size()) + ").");
    }

    // get position and rotation
    chrono::ChVector<double> position{0.0, 0.0, 0.0};
    chrono::ChQuaternion<double> rotation{0.0, 0.0, 0.0, 0.0};
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

std::vector<chrono::ChVector<double>> ComponentElastoFEA::get_nodes_positions() const {
    std::vector<chrono::ChVector<double>> positions;
    for (auto& node : nodes) {
        positions.push_back(node->GetPos());
    }
    return positions;
}

std::vector<chrono::ChVector<double>> ComponentElastoFEA::get_nodes_velocities() const {
    std::vector<chrono::ChVector<double>> velocities;
    for (auto& node : nodes) {
        velocities.push_back(node->GetPos_dt());
    }
    return velocities;
}

std::vector<chrono::ChVector<double>> ComponentElastoFEA::get_nodes_accelerations() const {
    std::vector<chrono::ChVector<double>> accelerations;
    for (auto& node : nodes) {
        accelerations.push_back(node->GetPos_dtdt());
    }
    return accelerations;
}

std::vector<chrono::ChQuaternion<double>> ComponentElastoFEA::get_nodes_rotations() const {
    std::vector<chrono::ChQuaternion<double>> rotations;
    for (auto& node : nodes) {
        rotations.push_back(node->GetRot());
    }
    return rotations;
}

std::vector<chrono::ChVector<double>> ComponentElastoFEA::get_nodes_directions() const {
    std::vector<chrono::ChVector<double>> directions;
    for (auto& node : nodes) {
        directions.push_back(node->GetRot().GetVector());
    }
    return directions;
}

std::vector<chrono::ChVector<double>> ComponentElastoFEA::get_nodes_rotational_velocities() const {
    std::vector<chrono::ChVector<double>> rotational_velocities;
    for (auto& node : nodes) {
        rotational_velocities.push_back(node->GetWvel_loc());
    }
    return rotational_velocities;
}

std::vector<chrono::ChVector<double>> ComponentElastoFEA::get_nodes_rotational_accelerations() const {
    std::vector<chrono::ChVector<double>> rotational_accelerations;
    for (auto& node : nodes) {
        rotational_accelerations.push_back(node->GetWacc_loc());
    }
    return rotational_accelerations;
}

std::vector<chrono::ChVector<double>> ComponentElastoFEA::get_nodes_loads() const {
    std::vector<chrono::ChVector<double>> loads;
    for (auto& node : nodes) {
        loads.push_back(node->GetForce());
    }
    return loads;
}
