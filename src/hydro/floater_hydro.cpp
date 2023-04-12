#include "seahowl/hydro/floater_hydro.h"

#include "seahowl/elasto/chrono_adapters.h"

#include <chrono/physics/ChBody.h>

using namespace seahowl::hydro;

FloaterHydroChrono::FloaterHydroChrono(){};

void FloaterHydroChrono::add_body(std::string& name) {
    bodies_map[name] = std::make_unique<seahowl::elasto::BodyElastoChrono>();
}

seahowl::elasto::BodyElasto& FloaterHydroChrono::get_body(std::string& name) {
    return *bodies_map[name];
}

void FloaterHydroChrono::initialize(double time, double dt) {
    if (bodies_map.size() < 1) {
        throw std::runtime_error("List of bodies for floater was not initialized.");
    } else if (h5_filepath == "") {
        throw std::runtime_error("Path of h5 file for floater was not defined.");
    }
    std::vector<std::shared_ptr<chrono::ChBody>> chbodies = {};
    for (auto& body : bodies_map) {
        auto& chbody = dynamic_cast<seahowl::elasto::BodyElastoChrono&>(*body.second);
        chbody.chobj->SetName(body.first.c_str());
        chbodies.push_back(chbody.chobj);
    }
    hydrochrono_setter = TestHydro(chbodies, h5_filepath, hydro_inputs);
}
