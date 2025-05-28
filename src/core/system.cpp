#include "seahowl/core/system.h"

#include "seahowl/elasto/system_elasto.h"
#include "seahowl/aero/system_aero.h"
#include "seahowl/elasto/mooring_elasto.h"
#include "seahowl/elasto/turbine_elasto.h"
#include "seahowl/aero/turbine_aero.h"
#include "seahowl/env/wind_models.h"
#include "seahowl/env/wave_models.h"
#include "seahowl/env/soil_models.h"
#include "seahowl/env/fluid_models.h"
#include "seahowl/core/blade.h"
#include "seahowl/elasto/blade_elasto.h"
#include "seahowl/elasto/floater_elasto.h"
#ifdef HAVE_HYDROCHRONO
    #include "seahowl/hydro/hydrochrono_adapter.h"
#endif

#include <vector>
#include <spdlog/spdlog.h>
#include <spdlog/stopwatch.h>
#include <map>
#include <iostream>

using namespace seahowl::core;
using namespace seahowl::env;

System::System(std::shared_ptr<seahowl::elasto::SystemElasto> elasto, std::shared_ptr<seahowl::aero::SystemAero> aero)
    : ComponentDynamic(elasto, aero), elasto(*elasto), aero(*aero){};

void System::build() {
    // build all turbines
    for (auto& turbine : turbines) {
        turbine->build();
    }
    // build all extra components
    for (auto& component : components) {
        component->build();
    }
}

void System::initialize_this(double time, double dt) {
    // assemble elasto system if it hasn't been already
    if (!elasto.is_assembled) {
        elasto.assemble();
    } else {
        spdlog::warn("Elasto system was already assembled, not reassembling.");
    }

#ifdef HAVE_HYDROCHRONO
    /// @todo replace this check with better handling (e.g. at initialization of floater by passing fluid model)
    // Specific HydroChrono handling: need to pass waves from environment to HydroChrono floater.
    // Needs to happen before initializing turbines

    for (auto& turbine : turbines) {
        if (turbine->elasto.foundation) {
            try {
                auto& floater = dynamic_cast<seahowl::hydro::FloaterHydroChrono&>(*turbine->elasto.foundation);
                auto wave_models = env_model->fluid_models.get_models_of_type<WaveModelHydroChrono>();
                if (wave_models.size() > 0) {
                    spdlog::debug("Passing waves to HydroChrono floater model.");
                    floater.set_waves_hydrochrono(wave_models[0]->waves);
                } else {
                    throw std::runtime_error("Must use HydroChrono wave model when using HydroChrono floater.");
                }
            } catch (const std::bad_cast& e) {
                // do nothing if foundation is not a HydroChrono floater
            }
        }
    }
    // also need to check components
    for (auto& component : components) {
        try {
            auto& floater_core = dynamic_cast<seahowl::core::Floater&>(*component);
            auto& floater = dynamic_cast<seahowl::hydro::FloaterHydroChrono&>(floater_core.elasto);
            auto wave_models = env_model->fluid_models.get_models_of_type<WaveModelHydroChrono>();
            if (wave_models.size() > 0) {
                spdlog::debug("Passing waves to HydroChrono floater model.");
                floater.set_waves_hydrochrono(wave_models[0]->waves);
            } else {
                throw std::runtime_error("Must use HydroChrono wave model when using HydroChrono floater.");
            }
        } catch (const std::bad_cast& e) {
            // do nothing if component is not a HydroChrono floater
        }
    }
#endif

    // initialize all turbines
    for (auto& turbine : turbines) {
        turbine->initialize(time, dt);
    }
    // initialize all extra components
    for (auto& component : components) {
        component->initialize(time, dt);
    }

    // check if fluid model exists
    if (!env_model->fluid_models.has_model()) {
        spdlog::warn("No fluid model was attached to the system.");
    }

    // check if soil model exists
    if (!env_model->soil_models.has_model()) {
        spdlog::warn("No soil model was attached to the system.");
    }

    spdlog::info("Initialized system with number of turbines: {}.", turbines.size());
}

void System::prestep(double time, double dt) {
    // apply control first
    for (auto& turbine : turbines) {
        turbine->apply_control(time, dt);
    }

    // compute forces from env model
    if (env_model->fluid_models.has_model()) {
        apply_env_model(*env_model, time);
    }

    // prestep (accumulates loads from aero to elasto)
    for (auto& turbine : turbines) {
        // turbine prestep
        turbine->prestep(time, dt);
    }
    for (auto& component : components) {
        // component prestep
        component->prestep(time, dt);
    }

    // compute forces from soil model
    // happens after prestep because soil loads directly applied to elasto component (e.g. moorings)
    if (env_model->soil_models.has_model()) {
        apply_soil_model(*env_model, time);
    }
}

void System::step(double dt) {
    elasto.step(dt);
}

void System::poststep(double time, double dt) {
    for (auto& turbine : turbines) {
        // turbine poststep
        turbine->poststep(time, dt);
    }
    for (auto& component : components) {
        // component poststep
        component->poststep(time, dt);
    }
}

void System::apply_env_model(seahowl::env::EnvModel& env_model, double time) {
    for (auto& turbine : turbines) {
        // compute forces from fluid model
        turbine->apply_env_model(env_model, time);
    }
    for (auto& component : components) {
        // compute forces from fluid model
        component->apply_env_model(env_model, time);
    }
}

void System::apply_soil_model(seahowl::env::EnvModel& env_model, double time) {
    for (auto& turbine : turbines) {
        // compute forces from fluid model
        turbine->apply_soil_model(env_model, time);
    }
    for (auto& component : components) {
        // compute forces from fluid model
        component->apply_soil_model(env_model, time);
    }
}

double System::get_time() const {
    return elasto.get_time();
}

void System::set_time(double time) {
    elasto.set_time(time);
}

void System::run_presimulation(double duration, double dt, bool fix_foundations, bool with_presetup) {
    if (duration == 0.0 || dt == 0.0 || dt > duration) {
        // do nothing if dt or number of steps is zero
        return;
    }
    int nsteps = int(duration / dt);
    spdlog::info("Presimulation of simulation for {} steps with dt={} and presetup as {}.", nsteps, dt, with_presetup);
    spdlog::stopwatch sw_presimulation;

    double time_init = get_time();

    std::vector<bool> foundation_was_fixed;
    for (int idx_turbine = 0; idx_turbine < turbines.size(); idx_turbine++) {
        auto& turbine = *turbines[idx_turbine];
        foundation_was_fixed.push_back(turbine.elasto.foundation->is_fixed());
        if (fix_foundations) {
            turbine.elasto.foundation->set_fixed(true);
        }
    }

    // apply length increments dynamically
    int step_frac = int(nsteps / 20.0);
    std::cout << "|0% -----------> 100%|" << std::endl;
    std::cout << "|";
    for (int step = 0; step <= nsteps; step++) {
        if (step > 0 && step % step_frac == 0) {
            std::cout << "*";
        }
        if (with_presetup) {
            elasto.presetup(double(step) / double(nsteps));
        }

        // stepping
        prestep(time_init, dt);
        elasto.step(dt);
        poststep(time_init, dt);

        // reset time
        set_time(time_init);
    }
    std::cout << "|" << std::endl;

    // unfix towers
    for (int idx_turbine = 0; idx_turbine < turbines.size(); idx_turbine++) {
        auto& turbine = *turbines[idx_turbine];
        if (foundation_was_fixed[idx_turbine]) {
            turbines[idx_turbine]->elasto.foundation->set_fixed(true);
        } else {
            turbines[idx_turbine]->elasto.foundation->set_fixed(false);
        }
    }

    spdlog::info("Presimulation finished{:.3}s.", sw_presimulation);
}

void System::add(std::shared_ptr<Turbine> turbine) {
    turbines.push_back(turbine);
}

void System::add(std::shared_ptr<ComponentDynamic> component) {
    components.push_back(component);
}
