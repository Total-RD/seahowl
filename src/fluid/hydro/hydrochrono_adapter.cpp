#include "seahowl/fluid/hydro/hydrochrono_adapter.h"

#include "seahowl/elasto/chrono_adapters.h"

#include <hydroc/hydro_forces.h>
#include <chrono/physics/ChBody.h>

#include <spdlog/spdlog.h>

using namespace seahowl::hydro;
using namespace seahowl::env;

FloaterHydroChrono::FloaterHydroChrono(){};

void FloaterHydroChrono::initialize() {
    if (h5_filepath == "") {
        throw std::runtime_error("Path of h5 file for HydroChrono floater was not defined.");
    } else if (floater_bodies.size() == 0) {
        throw std::runtime_error("Must add bodies to floater before initializing.");
    }

    if (!waves) {
        throw std::runtime_error("Need to attach waves to floater before initializing when using HydroChrono.");
    }

    // give names to Chrono bodies
    std::vector<std::shared_ptr<chrono::ChBody>> chbodies;
    for (auto& bodymap : floater_bodies) {
        auto& chbody = dynamic_cast<seahowl::elasto::BodyElastoChrono&>(*bodymap.second);
        chbody.chobj->SetName(bodymap.first.c_str());
        chbodies.push_back(chbody.chobj);
    }

    // send ChBody list to HydroChrono
    hydrochrono_setter = std::make_shared<TestHydro>(chbodies, h5_filepath);
    hydrochrono_setter->AddWaves(waves);
}

void FloaterHydroChrono::set_h5_filepath(const std::string& filepath) {
    h5_filepath = filepath;
}

void FloaterHydroChrono::set_waves_hydrochrono(std::shared_ptr<WaveBase> waves) {
    this->waves = waves;
}

void FloaterHydroChrono::set_waves(std::shared_ptr<seahowl::env::WaveModelHydroChrono> waves) {
    this->waves = waves->waves;
};

WaveModelHydroChrono::WaveModelHydroChrono() : waves(std::make_shared<NoWave>()) {}

seahowl::Vector3d WaveModelHydroChrono::get_velocity_this(const Vector3d& position, double time) const {
    if (is_inside(position, time)) {
        if (position.dot(surface_normal) < -waves->water_depth_ + 1e-6) {
            // return 0.0 if below soil level
            return Vector3d(0.0, 0.0, 0.0);
        }
        return waves->GetVelocity(position, time);
    } else {
        throw std::runtime_error("Cannot retrieve water velocity above mean water level.");
    }
}

seahowl::Vector3d WaveModelHydroChrono::get_acceleration_this(const Vector3d& position, double time) const {
    if (is_inside(position, time)) {
        if (position.dot(surface_normal) < -waves->water_depth_ + 1e-6) {
            // return 0.0 if below soil level
            return Vector3d(0.0, 0.0, 0.0);
        }
        return waves->GetAcceleration(position, time);
    } else {
        throw std::runtime_error("Cannot retrieve water acceleration above mean water level.");
    }
}

double WaveModelHydroChrono::get_density_this(const Vector3d& position, double time) const {
    return density;
}

double WaveModelHydroChrono::get_water_level(const Vector3d& position, double time) const {
    return waves->mwl_ + waves->GetElevation(position, time);
}
