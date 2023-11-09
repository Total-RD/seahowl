#include "seahowl/core/system.h"

#include "seahowl/elasto/system_elasto.h"
#include "seahowl/aero/system_aero.h"
#include "seahowl/elasto/mooring_elasto.h"
#include "seahowl/elasto/turbine_elasto.h"
#include "seahowl/aero/turbine_aero.h"
#include "seahowl/env/wind_models.h"
#include "seahowl/core/turbine_floating.h"
#include "seahowl/core/blade.h"
#include "seahowl/elasto/blade_elasto.h"

#include <vector>
#include <spdlog/spdlog.h>
#include <map>
#include <iostream>

using namespace seahowl::core;

System::System() {}

void System::initialize(double time, double dt) {
    for (auto& turbine : turbines) {
        turbine->initialize(time, dt);
    }

    if (!fluid_model) {
        spdlog::warn("No fluid model was attached to the system.");
    }
    if (!soil_model) {
        spdlog::warn("No soil model was attached to the system.");
    }
    spdlog::info("Initialized system with number of turbines: {}.", turbines.size());
}

void System::prestep(double time, double dt) {
    for (auto& turbine : turbines) {
        // compute forces from fluid model
        if (fluid_model) {
            turbine->apply_fluid_model(*fluid_model, time);
        }
        // compute forces from soil model
        if (soil_model) {
            turbine->apply_soil_model(*soil_model, time);
        }
        // turbine prestep (accumulates loads from aero to elasto)
        turbine->prestep(time, dt);
    }
}

void System::step(double dt) {
    system_elasto->step(dt);
}

void System::poststep(double time, double dt) {
    for (auto& turbine : turbines) {
        // turbine poststep
        turbine->poststep(time, dt);
    }
}

void System::assemble() {
    for (auto& turbine : turbines) {
        turbine->elasto.assemble(*(system_elasto.get()));
    }
}

double System::get_time() {
    return system_elasto->get_time();
}

void System::set_time(double time) {
    system_elasto->set_time(time);
}

void System::presetup(double dt, int nsteps) {
    spdlog::info("Presetup of simulation for {} steps with dt={}.", nsteps, dt);

    std::map<int, std::vector<std::vector<double>>> lengths_initial_moorings;
    std::map<int, std::vector<std::vector<double>>> lengths_final_moorings;
    std::map<int, std::vector<std::vector<double>>> lengths_delta_moorings;
    for (int idx_turbine = 0; idx_turbine < turbines.size(); idx_turbine++) {
        auto& turbine = *turbines[idx_turbine];
        try {
            auto& turbine_floating = dynamic_cast<TurbineFloating&>(turbine);

            // fix turbine
            turbine_floating.tower.elasto.nodes.front()->set_fixed(true);
            turbine_floating.rna.elasto.rotor->body_hub->set_fixed(true);

            for (auto& mooring_ptr : turbine_floating.elasto.mooring_system->moorings) {
                auto& mooring_elasto = *mooring_ptr;
                auto& mooring = dynamic_cast<seahowl::elasto::MooringElastoFEA&>(mooring_elasto);

                // check that mooring was built properly
                int nb_elements = mooring.elements.size();
                if (nb_elements != mooring.discretization_fractions.size() - 1) {
                    throw std::runtime_error(
                        "Number of elements and discretization fractions on mooring do not match (build mooring "
                        "first?).");
                }

                // get initial and final lengths of mooring, and length increment to apply
                std::vector<double> lengths_initial(nb_elements);
                std::vector<double> lengths_final(nb_elements);
                std::vector<double> lengths_delta(nb_elements);

                auto distance_fairlead_anchor =
                    (mooring.fairlead.get_position() - mooring.anchor.get_position()).norm();
                auto density_equivalent = mooring.density_linear * (mooring.length / distance_fairlead_anchor) /
                                          (seahowl::PI * pow(mooring.diameter / 2.0, 2));
                for (size_t idx_el = 0; idx_el < nb_elements; idx_el++) {
                    auto& element = dynamic_cast<seahowl::elasto::ElementMooringElasto&>(*mooring.elements[idx_el]);
                    lengths_final[idx_el] = mooring.length * (mooring.discretization_fractions[idx_el + 1] -
                                                              mooring.discretization_fractions[idx_el]);
                    lengths_initial[idx_el] =
                        (element.nodes[1]->get_position() - element.nodes[0]->get_position()).norm();
                    lengths_delta[idx_el] = (lengths_final[idx_el] - lengths_initial[idx_el]) / nsteps;
                    // initializing element with density-equivalent length
                    element.set_properties(density_equivalent, mooring.diameter, mooring.stiffness_axial,
                                           mooring.stiffness_bending);
                }

                lengths_initial_moorings[idx_turbine].push_back(lengths_initial);
                lengths_final_moorings[idx_turbine].push_back(lengths_final);
                lengths_delta_moorings[idx_turbine].push_back(lengths_delta);
            }
        } catch (const std::bad_cast& e) {
            // do nothing
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
        for (int idx_turbine = 0; idx_turbine < turbines.size(); idx_turbine++) {
            auto& turbine = *turbines[idx_turbine];
            try {
                auto& turbine_floating = dynamic_cast<TurbineFloating&>(turbine);
                for (int idx_mooring = 0; idx_mooring < turbine_floating.elasto.mooring_system->moorings.size();
                     idx_mooring++) {
                    auto& mooring_elasto = *turbine_floating.elasto.mooring_system->moorings[idx_mooring];
                    auto& mooring = dynamic_cast<seahowl::elasto::MooringElastoFEA&>(mooring_elasto);
                    auto lengths_initial = lengths_initial_moorings[idx_turbine][idx_mooring];
                    auto lengths_delta = lengths_delta_moorings[idx_turbine][idx_mooring];
                    int nb_elements = mooring.elements.size();
                    for (int idx_el = 0; idx_el < nb_elements; idx_el++) {
                        auto& element = dynamic_cast<seahowl::elasto::ElementMooringElasto&>(*mooring.elements[idx_el]);
                        element.set_rest_length(lengths_initial[idx_el] + lengths_delta[idx_el] * step);
                        if (step == 1) {
                            // reset to actual properties after initializing with density-equivalent length
                            element.set_properties(
                                mooring.density_linear / (seahowl::PI * pow(mooring.diameter / 2.0, 2)),
                                mooring.diameter, mooring.stiffness_axial, mooring.stiffness_bending);
                        }
                    }
                    mooring.compute_hydro_loads(system_elasto->get_gravitational_acceleration(), 1000.);

                    if (soil_model) {
                        mooring.compute_seabed_loads(*soil_model);
                    }
                }
            } catch (const std::bad_cast& e) {
                // do nothing
            }
        }

        system_elasto->step(dt);
    }

    std::cout << "|" << std::endl;
    // unfix floating turbine
    for (int idx_turbine = 0; idx_turbine < turbines.size(); idx_turbine++) {
        auto& turbine = *turbines[idx_turbine];
        try {
            auto& turbine_floating = dynamic_cast<TurbineFloating&>(turbine);
            turbine_floating.tower.elasto.nodes.front()->set_fixed(false);
            turbine_floating.rna.elasto.rotor->body_hub->set_fixed(false);
        } catch (const std::bad_cast& e) {
            // do nothing
        }
    }

    // reset time
    set_time(0.0);

    spdlog::info("Presetup finished.");
}
