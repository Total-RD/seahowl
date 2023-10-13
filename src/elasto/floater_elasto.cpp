#include "seahowl/elasto/floater_elasto.h"

#include "seahowl/elasto/chrono_adapters.h"

using namespace seahowl::elasto;

FloaterElasto::FloaterElasto(){};

seahowl::elasto::BodyElasto& FloaterElasto::add_body(std::string& name) {
    floater_bodies[name] = std::make_unique<seahowl::elasto::BodyElastoChrono>();
    return *floater_bodies[name];
}

seahowl::elasto::BodyElasto& FloaterElasto::get_body(std::string& name) {
    return *floater_bodies[name];
}

void FloaterElasto::add_fairlead(Vector3d& position, std::string connected_body_name) {
    if (floater_bodies.count(connected_body_name) == 0) {
        throw std::runtime_error("Trying to add a fairlead to a non-existing floater body " + connected_body_name);
    }

    // fairlead
    fairlead_bodies[connected_body_name].push_back(std::unique_ptr<seahowl::elasto::BodyElastoChrono>());
    auto& fairlead = *(fairlead_bodies[connected_body_name].back());
    fairlead.set_position(position);

    // link
    fairlead_links[connected_body_name].push_back(std::unique_ptr<seahowl::elasto::LinkChrono>());
    auto& link = *(fairlead_links[connected_body_name].back().get());
    link.set_constraints(true, true, true, false, false, false);
    link.initialize(fairlead, *floater_bodies[connected_body_name]);
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

void FloaterElasto::set_tower_connection_body_name(std::string connected_body_name) {
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
