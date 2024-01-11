#include "seahowl/elasto/blade_elasto.h"

#include "seahowl/elasto/chrono_adapters.h"
#include "seahowl/commons/utils.h"  // For DiscretizationPoint
#include "seahowl/elasto/reference_point_elasto.h"

#include <numeric>
#include <spdlog/spdlog.h>

using namespace seahowl::elasto;

BladeElasto::BladeElasto() {
    discretization_fractions = {0.0, 1.0};
    link_root = std::make_unique<LinkChrono>();
    link_root->set_constraints(true, true, true, true, true, true);
}

void BladeElasto::assemble(SystemElasto& system) {
    if (is_mounted) {
        system.add(*(link_root.get()));
    }
}

BladeElastoFEA::BladeElastoFEA() {}

void BladeElastoFEA::assemble(SystemElasto& system) {
    BladeElasto::assemble(system);
    ComponentElastoFEA::assemble(system);
}

void BladeElastoFEA::build() {
    // check that enough reference points were defined to create elements (at least 2)
    if (reference_points.size() < 2) {
        throw std::runtime_error("Not enough elasto reference points defined for blade (" +
                                 std::to_string(reference_points.size()) + ").");
    }

    // check that discretization_fractions was defined, otherwise take reference point fractions
    if (discretization_fractions.size() == 0) {
        for (int ii = 0; ii < reference_points.size(); ii++) {
            discretization_fractions.push_back(reference_points[ii].fraction);
        }
    } else if (discretization_fractions.size() == 1) {
        double npoints = discretization_fractions[0] + 1;
        double dp = 1.0 / (npoints - 1);
        discretization_fractions.clear();
        for (int ii = 0; ii < int(npoints); ii++) {
            discretization_fractions.push_back(ii * dp);
        }
    }

    // build
    discretized_points = seahowl::get_discretized_points(discretization_fractions, reference_points);
    ///@todo find better way to build ReferencePointElasto from BladeReferencePointElasto
    std::vector<ReferencePointElasto> discretized_points0;
    for (int ii = 0; ii < discretized_points.size(); ii++) {
        auto discretized_point0 = ReferencePointElasto();
        discretized_point0.coordinates = discretized_points[ii].coordinates;
        discretized_point0.fraction = discretized_points[ii].fraction;
        discretized_points0.push_back(discretized_point0);
    }
    // build nodes
    build_nodes(discretized_points0);
    // apply properties
    for (int ii = 0; ii < nodes.size(); ii++) {
        std::dynamic_pointer_cast<NodeElastoChrono>(nodes[ii])->set_properties(discretized_points[ii], fpm_mode);
    }
    // apply structural twist
    for (int ii = 0; ii < nodes.size(); ii++) {
        auto& node = nodes[ii];
        auto& point = discretized_points[ii];
        auto axis = node->get_direction();
        auto twist_matrix = AngleAxisd(-point.structural_twist, axis);
        auto rotation_matrix = twist_matrix * nodes[ii]->get_rotation().toRotationMatrix();
        nodes[ii]->set_rotation(Quaternion(rotation_matrix));
    }

    if (fpm_mode) {
        build_elements_tapered_timoshenko_fpm();
    } else {
        build_elements_tapered_timoshenko();
    }
};

void BladeElastoFEA::build_elements_tapered_timoshenko() {
    elements.clear();
    const auto nelements = nodes.size() - 1;

    if (nelements <= 0) {
        throw std::runtime_error("Trying to build blade with no element.");
    }

    for (size_t ii = 1; ii < nelements + 1; ii++) {
        // create element
        auto element = std::make_shared<ElementBladeElastoChrono>();
        // add element to blade elements vector
        elements.push_back(element);
        // set element nodes
        element->set_nodes(nodes[ii - 1], nodes[ii]);

        // apply prebend and structural twist
        auto rotation_relative = (nodes[ii]->get_rotation() * nodes[ii - 1]->get_rotation().inverse()).normalized();
        element->set_prebend(rotation_relative);
    }
}

void BladeElastoFEA::build_elements_tapered_timoshenko_fpm() {
    elements.clear();
    const auto nelements = nodes.size() - 1;

    if (nelements <= 0) {
        throw std::runtime_error("Trying to build blade with no element.");
    }

    for (size_t ii = 1; ii < nelements + 1; ii++) {
        // create element
        auto element = std::make_shared<ElementBladeElastoChronoFPM>();
        // add element to blade elements vector
        elements.push_back(element);
        // set element nodes
        element->set_nodes(nodes[ii - 1], nodes[ii]);

        // apply prebend and structural twist
        auto rotation_relative = (nodes[ii]->get_rotation() * nodes[ii - 1]->get_rotation().inverse()).normalized();
        element->set_prebend(rotation_relative);
    }
}

void BladeElastoFEA::evaluate_position_rotation(Vector3d& position,
                                                Quaternion& rotation,
                                                int element_index,
                                                double eta) const {
    elements[element_index]->evaluate_position_rotation(eta, position, rotation);
}

void BladeElastoFEA::apply_pitch_increment(double pitch_increment) {
    // apply pitch from root node direction and position
    auto root_dir = nodes.front()->get_direction();
    auto root_pos = nodes.front()->get_position();
    translate(-root_pos);
    rotate(-pitch_increment, root_dir);
    translate(root_pos);
    pitch += pitch_increment;
}

seahowl::Vector3d BladeElastoFEA::get_blade_root_moment() const {
    return elements[0]->get_torque(-1.0);
}

seahowl::EntityDynamicEigen BladeElastoFEA::get_entity_along_blade(double eta, int element_index) const {
    return get_entity_along_component(eta, element_index);
}

void BladeElastoFEA::accumulate_load_along_blade(const seahowl::Vector3d& load,
                                                 const seahowl::Vector3d& moment,
                                                 int element_index,
                                                 double eta,
                                                 const seahowl::Vector3d& offset) {
    accumulate_element_load(load, moment, element_index, eta, offset);
}

void BladeElastoFEA::attach_root_to_body(const BodyElasto& body) {
    is_mounted = true;
    link_root->initialize(*(nodes.front().get()), body);
};

BladeElastoRigid::BladeElastoRigid() {}

void BladeElastoRigid::build() {
    body_root = std::make_unique<BodyElastoChrono>();
    length = reference_points.back().coordinates.z();

    // calculate mass
    double mass_total = 0.0;
    double inertia_total = 0.0;
    for (int ii = 0; ii < reference_points.size() - 1; ii++) {
        auto& point1 = reference_points[ii];
        auto& point2 = reference_points[ii + 1];
        auto rho1 = point1.mass_matrix(2, 2);
        auto rho2 = point2.mass_matrix(2, 2);
        auto l1 = point1.coordinates.z();
        auto l2 = point2.coordinates.z();
        auto length_segment = abs(l2 - l1);

        // mass of blade segment
        mass_total += 0.5 * (rho1 + rho2) * length_segment;

        // inertia assuming rod element with non-uniform linear density
        inertia_total += ((pow(l2, 4) - pow(l1, 4)) / 4.0 * (rho2 - rho1) +
                          (pow(l2, 3) - pow(l1, 3)) / 3.0 * (rho1 * l2 - rho2 * l1)) /
                         length_segment;
    };

    // set mass and inertia at root
    body_root->set_mass(mass_total);
    // inertia of blade calculated from body root for rotation along local x and y
    body_root->set_inertia_diagonal(Vector3d(inertia_total, inertia_total, 0.0));
}

void BladeElastoRigid::assemble(SystemElasto& system) {
    BladeElasto::assemble(system);
    system.add(*(body_root.get()));
}

void BladeElastoRigid::rotate(double angle, const Vector3d& axis) const {
    auto rotation = AngleAxisd(angle, axis);
    // blade root
    auto new_position_root = rotation * body_root->get_position();
    auto new_rotation_root = (rotation * body_root->get_rotation()).normalized();
    body_root->set_position(new_position_root);
    body_root->set_rotation(new_rotation_root);
}

void BladeElastoRigid::translate(const Vector3d& translation_vector) const {
    // blade root
    body_root->set_position(body_root->get_position() + translation_vector);
}

double BladeElastoRigid::get_mass() const {
    return body_root->get_mass();
}

void BladeElastoRigid::apply_pitch_increment(double pitch_increment) {
    // update rotation around local Z-axis
    auto root_dir = body_root->get_rotation() * Vector3d(0.0, 0.0, 1.0);
    auto root_pos = body_root->get_position();
    translate(-root_pos);
    rotate(-pitch_increment, root_dir);
    translate(root_pos);
    pitch += pitch_increment;
}

seahowl::Vector3d BladeElastoRigid::get_blade_root_moment() const {
    return body_root->get_torque();
}

seahowl::EntityDynamicEigen BladeElastoRigid::get_entity_along_blade(double eta, int element_index) const {
    auto entity = seahowl::EntityDynamicEigen();

    // relative position along z axis of blade
    auto z_position = (0.5 * fabs(eta + 1.0)) * length;

    // position
    Vector3d position = body_root->get_position() + body_root->get_rotation() * Vector3d(0.0, 0.0, z_position);
    entity.set_position(position);

    // get structural twist
    std::vector<double> eta_vector = {0.5 * (eta + 1.0)};
    auto twist = seahowl::get_discretized_points(eta_vector, reference_points)[0].structural_twist;

    // rotation
    auto twist_matrix = AngleAxisd(-twist, body_root->get_rotation() * Vector3d(0.0, 0.0, 1.0));
    auto rotation_matrix = twist_matrix * body_root->get_rotation().toRotationMatrix();
    entity.set_rotation(Quaternion(rotation_matrix));

    // below has to be explicitly declared as Vector3d or there is an issue;
    Vector3d pos1 = entity.get_position();
    Vector3d radius_vector = (pos1 - body_root->get_position());

    // velocity
    Vector3d velocity = body_root->get_velocity() + (body_root->get_rotational_velocity(false)).cross(radius_vector);
    entity.set_velocity(velocity);
    entity.set_rotational_velocity(body_root->get_rotational_velocity());

    // acceleration
    Vector3d acceleration =
        body_root->get_acceleration() + (body_root->get_rotational_acceleration(false)).cross(radius_vector);
    entity.set_acceleration(acceleration);
    entity.set_rotational_acceleration(body_root->get_rotational_acceleration());

    return entity;
}

void BladeElastoRigid::reset_loads() {
    body_root->reset_loads();
}

void BladeElastoRigid::accumulate_load_along_blade(const seahowl::Vector3d& load,
                                                   const seahowl::Vector3d& moment,
                                                   int element_index,
                                                   double eta,
                                                   const seahowl::Vector3d& offset) {
    // apply force on root
    body_root->accumulate_force(load, false);

    // apply moment on root
    auto entity = get_entity_along_blade(eta, element_index);
    auto distance = (entity.get_position() + offset - body_root->get_position());
    body_root->accumulate_torque(moment + distance.cross(load), false);
}

void BladeElastoRigid::attach_root_to_body(const BodyElasto& body) {
    is_mounted = true;
    link_root->initialize(*body_root, body);
};
