#include "seahowl/elasto/floater_elasto.h"

#include "seahowl/elasto/chrono_adapters.h"

#include <spdlog/spdlog.h>

using namespace seahowl::elasto;

FloaterElasto::FloaterElasto(){};

void FloaterElasto::prestep(double time, double dt) {
    auto& floater_body = get_tower_connection_body();
    floater_body.reset_loads();

    // apply viscous damping
    // get local velocity (damping matrix expressed in local frame)
    auto velocity_local = floater_body.get_rotation().inverse() * floater_body.get_velocity();
    auto velocity_rot_local = floater_body.get_rotational_velocity(true);
    // make vector of length 6
    Eigen::Matrix<double, 6, 1> floater_velocity_vector(6);
    floater_velocity_vector << velocity_local.x(), velocity_local.y(), velocity_local.z(), velocity_rot_local.x(),
        velocity_rot_local.y(), velocity_rot_local.z();
    // calculate damping force and moment
    auto damping_vector = damping_matrix * (-floater_velocity_vector);
    // project in global frame
    auto damping_force =
        floater_body.get_rotation() * seahowl::Vector3d(damping_vector(0), damping_vector(1), damping_vector(2));
    auto damping_torque =
        floater_body.get_rotation() * seahowl::Vector3d(damping_vector(3), damping_vector(4), damping_vector(5));
    // apply damping
    floater_body.accumulate_force(damping_force, false);
    floater_body.accumulate_torque(damping_torque, false);
}

seahowl::elasto::BodyElasto& FloaterElasto::add_body(const std::string& name) {
    floater_bodies[name] = std::make_unique<seahowl::elasto::BodyElastoChrono>();
    return *floater_bodies[name];
}

seahowl::elasto::BodyElasto& FloaterElasto::get_body(const std::string& name) {
    return *floater_bodies[name];
}

void FloaterElasto::add_fairlead(const Vector3d& position, const std::string& connected_body_name) {
    if (floater_bodies.count(connected_body_name) == 0) {
        throw std::runtime_error("Trying to add a fairlead to a non-existing floater body " + connected_body_name +
                                 ".");
    }

    // fairlead
    fairlead_bodies[connected_body_name].push_back(std::make_unique<seahowl::elasto::BodyElastoChrono>());
    auto& fairlead = *(fairlead_bodies[connected_body_name].back());
    fairlead.set_position(position);

    // link
    fairlead_links[connected_body_name].push_back(std::make_unique<seahowl::elasto::LinkChrono>());
    auto& link = *(fairlead_links[connected_body_name].back().get());
    link.set_constraints(true, true, true, true, true, true);
    link.initialize(fairlead, *floater_bodies[connected_body_name]);
}

int FloaterElasto::get_fairlead_count(const std::string& body_name) const {
    if (floater_bodies.count(body_name) == 0) {
        throw std::runtime_error("Trying to get fairlead count from a non-existing floater body " + body_name + ".");
    }
    return fairlead_bodies.at(body_name).size();
};

seahowl::elasto::BodyElasto& FloaterElasto::get_fairlead_body(const std::string& body_name, int index) {
    if (floater_bodies.count(body_name) == 0) {
        throw std::runtime_error("Trying to get fairlead body from a non-existing floater body " + body_name + ".");
    }

    auto& fairleads_vector = fairlead_bodies.at(body_name);
    if (fairleads_vector.size() <= index) {
        throw std::runtime_error("Trying to access fairlead body " + std::to_string(index) + " of " + body_name +
                                 " but it has only " + std::to_string(fairleads_vector.size()) + " fairleads.");
    } else {
        return *fairleads_vector[index];
    }
}

seahowl::elasto::Link& FloaterElasto::get_fairlead_link(const std::string& body_name, int index) {
    if (floater_bodies.count(body_name) == 0) {
        throw std::runtime_error("Trying to get fairlead link from a non-existing floater body " + body_name + ".");
    }

    auto& fairleads_vector = fairlead_links.at(body_name);
    if (fairleads_vector.size() <= index) {
        throw std::runtime_error("Trying to access fairlead link " + std::to_string(index) + " of " + body_name +
                                 " but it has only " + std::to_string(fairleads_vector.size()) + " fairleads.");
    } else {
        return *fairleads_vector[index];
    }
}

void FloaterElasto::assemble(seahowl::elasto::SystemElasto& system) {
    for (auto& bodymap : floater_bodies) {
        auto& body = *bodymap.second;
        system.add(body);
    }
    for (auto& fairleadmap : fairlead_bodies) {
        for (auto& fairlead : fairleadmap.second) {
            system.add(*fairlead);
        }
    }
    for (auto& linkmap : fairlead_links) {
        for (auto& link : linkmap.second) {
            system.add(*link);
        }
    }
}

void FloaterElasto::set_tower_connection_body_name(const std::string& connected_body_name) {
    tower_connection_name = connected_body_name;
}

seahowl::elasto::BodyElasto& FloaterElasto::get_tower_connection_body() const {
    if (floater_bodies.size() < 1) {
        throw std::runtime_error("Need to add at least one body to floater before connecting to tower.");
    } else if (floater_bodies.size() == 1) {
        return *(floater_bodies.begin()->second);
    } else {
        if (tower_connection_name == "") {
            throw std::runtime_error("Need to set name of floater body that connects to tower.");
        } else {
            return *floater_bodies.at(tower_connection_name);
        }
    }
}

void FloaterElasto::translate(const Vector3d& translation_vector) const {
    // translate all bodies
    for (auto& bodymap : floater_bodies) {
        auto& body = *bodymap.second;
        body.set_position(body.get_position() + translation_vector);
    }
}

void FloaterElasto::rotate(double angle, const Vector3d& axis) const {
    // rotate all bodies
    auto rotation = AngleAxisd(angle, axis);
    for (auto& bodymap : floater_bodies) {
        auto& body = *bodymap.second;
        auto new_position_body = rotation * body.get_position();
        auto new_rotation_body = (rotation * body.get_rotation()).normalized();
        body.set_position(new_position_body);
        body.set_rotation(new_rotation_body);
    }
}

double FloaterElasto::get_mass() const {
    double total_mass = 0;
    for (auto& bodymap : floater_bodies) {
        auto& body = *bodymap.second;
        total_mass += body.get_mass();
    }
    for (auto& fairleadmap : fairlead_bodies) {
        for (auto& fairlead : fairleadmap.second) {
            total_mass += fairlead->get_mass();
        }
    }
    return total_mass;
}
