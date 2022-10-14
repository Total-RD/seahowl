#include <seahowl/elasto/elasto.h>

#include <vector>
#include <numeric>

#include <chrono/fea/ChElementBeamTaperedTimoshenko.h>
#include <chrono/fea/ChMesh.h>

using namespace seahowl::elasto;

void ElastoFEAComponent::assemble(std::shared_ptr<chrono::fea::ChMesh> mesh) const {
    for (auto node : nodes) {
        mesh->AddNode(node);
    }
    for (auto element : elements) {
        mesh->AddElement(element);
    }
}

void ElastoFEAComponent::rotate(double angle, chrono::ChVector<double> axis) const {
    auto rotation = Q_from_AngAxis(angle, axis);
    for (auto& node : nodes) {
        auto new_position = rotation.Rotate(node->GetPos());
        node->SetPos(new_position);
        auto new_rotation = (rotation * node->GetRot()).GetNormalized();
        node->SetRot(new_rotation);
    }
}

void ElastoFEAComponent::translate(chrono::ChVector<double> translation_vector) const {
    for (auto& node : nodes) {
        node->SetPos(node->GetPos() + translation_vector);
    }
}
double ElastoFEAComponent::get_mass() const {
    return std::accumulate(
        cbegin(elements), cend(elements), 0.0,
        [](double total, decltype(elements)::value_type pElem) { return total += pElem->GetMass(); });
}

void ElastoFEAComponent::reset_loads() {
    for (auto& node : nodes) {
        node->SetForce({0.0, 0.0, 0.0});
        node->SetTorque({0.0, 0.0, 0.0});
    }
}

void ElastoFEAComponent::evaluate_position_rotation(chrono::ChVector<double>& position,
                                                    chrono::ChQuaternion<double>& rotation,
                                                    int element_index,
                                                    double eta) const {
    auto& element = elements[element_index];

    element->EvaluateSectionFrame(eta, position, rotation);
}

void ElastoFEAComponent::accumulate_element_load(chrono::ChVector<double> load, int element_index, double eta) {
    if (element_index >= elements.size() || element_index < 0) {
        throw std::runtime_error("Element index " + std::to_string(element_index) + " does not exist (max " +
                                 std::to_string(elements.size()) + ").");
    }
    chrono::ChVector<double> position{0.0, 0.0, 0.0};
    chrono::ChQuaternion<double> rotation{0.0, 0.0, 0.0, 0.0};
    evaluate_position_rotation(position, rotation, element_index, eta);

    auto& element = elements[element_index];

    // load on first node
    double weight0 = 0.5 * abs(eta - 1);
    auto load0 = load * weight0;
    auto node0 = std::dynamic_pointer_cast<chrono::fea::ChNodeFEAxyzrot>(element->GetNodeN(0));
    node0->SetForce(node0->GetForce() + load0);
    node0->SetTorque(node0->GetTorque() + (position - node0->GetPos()) % load0);

    // load on second node
    double weight1 = 0.5 * abs(eta - 1);
    auto load1 = load * weight1;
    auto node1 = std::dynamic_pointer_cast<chrono::fea::ChNodeFEAxyzrot>(element->GetNodeN(1));
    node1->SetForce(node1->GetForce() + load1);
    node1->SetTorque(node1->GetTorque() + (position - node1->GetPos()) % load1);
}

std::vector<chrono::ChVector<double>> ElastoFEAComponent::get_nodes_positions() const {
    std::vector<chrono::ChVector<double>> positions;
    for (auto& node : nodes) {
        positions.push_back(node->GetPos());
    }
    return positions;
}

std::vector<chrono::ChVector<double>> ElastoFEAComponent::get_nodes_velocities() const {
    std::vector<chrono::ChVector<double>> velocities;
    for (auto& node : nodes) {
        velocities.push_back(node->GetPos_dt());
    }
    return velocities;
}

std::vector<chrono::ChVector<double>> ElastoFEAComponent::get_nodes_accelerations() const {
    std::vector<chrono::ChVector<double>> accelerations;
    for (auto& node : nodes) {
        accelerations.push_back(node->GetPos_dtdt());
    }
    return accelerations;
}

std::vector<chrono::ChQuaternion<double>> ElastoFEAComponent::get_nodes_rotations() const {
    std::vector<chrono::ChQuaternion<double>> rotations;
    for (auto& node : nodes) {
        rotations.push_back(node->GetRot());
    }
    return rotations;
}

std::vector<chrono::ChVector<double>> ElastoFEAComponent::get_nodes_directions() const {
    std::vector<chrono::ChVector<double>> directions;
    for (auto& node : nodes) {
        directions.push_back(node->GetRot().GetVector());
    }
    return directions;
}

std::vector<chrono::ChVector<double>> ElastoFEAComponent::get_nodes_rotational_velocities() const {
    std::vector<chrono::ChVector<double>> rotational_velocities;
    for (auto& node : nodes) {
        rotational_velocities.push_back(node->GetWvel_loc());
    }
    return rotational_velocities;
}

std::vector<chrono::ChVector<double>> ElastoFEAComponent::get_nodes_rotational_accelerations() const {
    std::vector<chrono::ChVector<double>> rotational_accelerations;
    for (auto& node : nodes) {
        rotational_accelerations.push_back(node->GetWacc_loc());
    }
    return rotational_accelerations;
}

std::vector<chrono::ChVector<double>> ElastoFEAComponent::get_nodes_loads() const {
    std::vector<chrono::ChVector<double>> loads;
    for (auto& node : nodes) {
        loads.push_back(node->GetForce());
    }
    return loads;
}