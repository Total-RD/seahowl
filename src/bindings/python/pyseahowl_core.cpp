// SPDX-License-Identifier: Apache-2.0
// Copyright 2022–2026 TotalEnergies
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//

// pybind11 headers
#include <pybind11/eigen.h>
#include <pybind11/pybind11.h>
#include <pybind11/stl.h>

// SEAHOWL headers
#include <seahowl/commons.h>
#include <seahowl/core.h>
#include <seahowl/elasto.h>
#include <seahowl/env.h>
#include <seahowl/fluid.h>
#include <seahowl/io.h>
#include <seahowl/servo.h>

namespace py = pybind11;

void initialize_pyseahowl_core(py::module& m) {
    // submodule
    auto m_core = m.def_submodule("core", "Core submodule.");

    // core/utils.h
    py::class_<seahowl::core::ComponentDynamic, std::shared_ptr<seahowl::core::ComponentDynamic>>(m_core,
                                                                                                  "ComponentDynamic")
        .def("build", &seahowl::core::ComponentDynamic::build)
        .def("initialize", &seahowl::core::ComponentDynamic::initialize)
        .def("prestep", &seahowl::core::ComponentDynamic::prestep)
        .def("poststep", &seahowl::core::ComponentDynamic::poststep)
        .def("apply_env_model", &seahowl::core::ComponentDynamic::apply_env_model)
        .def("apply_soil_model", &seahowl::core::ComponentDynamic::apply_soil_model);

    // core/simulation.h
    py::class_<seahowl::core::Simulation, std::shared_ptr<seahowl::core::Simulation>>(m_core, "Simulation")
        .def(py::init<>())
        .def("run_all", &seahowl::core::Simulation::run_all)
        .def("step", &seahowl::core::Simulation::step)
        .def("populate_from_file", &seahowl::core::Simulation::populate_from_file)
        .def("populate_from_config", &seahowl::core::Simulation::populate_from_config)
        .def("initialize", &seahowl::core::Simulation::initialize)
        .def("initialize_from_config", &seahowl::core::Simulation::initialize_from_config)
        .def_readwrite("dt", &seahowl::core::Simulation::dt)
        .def_readwrite("duration", &seahowl::core::Simulation::duration)
        .def_readonly("is_initialized", &seahowl::core::Simulation::is_initialized)
        .def_property_readonly(
            "system_core", [](seahowl::core::Simulation& sim) { return sim.system_core.get(); },
            py::return_value_policy::reference_internal)
        .def_property_readonly(
            "outputs", [](seahowl::core::Simulation& sim) { return sim.outputs.get(); },
            py::return_value_policy::reference_internal);

    // core/turbine.h
    py::class_<seahowl::core::Turbine, std::shared_ptr<seahowl::core::Turbine>, seahowl::core::ComponentDynamic>(
        m_core, "Turbine")
        .def(py::init<std::shared_ptr<seahowl::elasto::TurbineElasto>, std::shared_ptr<seahowl::fluid::TurbineFluid>>())
        .def("apply_control", &seahowl::core::Turbine::apply_control)
        .def("get_shaft_power", &seahowl::core::Turbine::get_shaft_power)
        .def("get_generated_power", &seahowl::core::Turbine::get_generated_power)
        .def("get_generator_rpm", &seahowl::core::Turbine::get_generator_rpm)
        .def_property_readonly("elasto", [](seahowl::core::Turbine& turbine) { return &turbine.elasto; })
        .def_property_readonly("fluid", [](seahowl::core::Turbine& turbine) { return &turbine.fluid; })
        .def_property_readonly(
            "aero",
            [](seahowl::core::Turbine& turbine) {
                seahowl::log("Turbine.aero is deprecated, use Turbine.fluid instead for aero/hydro access.", "warn");
                return &turbine.fluid;
            })
        .def_readonly("controller", &seahowl::core::Turbine::controller)
        .def_readonly("tower", &seahowl::core::Turbine::tower)
        .def_readonly("rna", &seahowl::core::Turbine::rna)
        .def_readonly("foundation", &seahowl::core::Turbine::foundation)
        .def_readwrite("generator_efficiency", &seahowl::core::Turbine::generator_efficiency)
        .def_readwrite("gearbox_ratio", &seahowl::core::Turbine::gearbox_ratio)
        .def_readwrite("gearbox_efficiency", &seahowl::core::Turbine::gearbox_efficiency);

    // core/tower.h
    py::class_<seahowl::core::Tower, std::shared_ptr<seahowl::core::Tower>, seahowl::core::ComponentDynamic>(m_core,
                                                                                                             "Tower")
        .def(py::init<std::shared_ptr<seahowl::elasto::TowerElasto>, std::shared_ptr<seahowl::aero::TowerAero>>())
        .def("set_discretization_elasto", &seahowl::core::Tower::set_discretization_elasto)
        .def("set_discretization_aero", &seahowl::core::Tower::set_discretization_aero)
        .def("update_positions_aero", &seahowl::core::Tower::update_positions_aero)
        .def("update_loads_elasto", &seahowl::core::Tower::update_loads_elasto)
        .def_property_readonly("elasto", [](seahowl::core::Tower& tower) { return &tower.elasto; })
        .def_property_readonly("aero", [](seahowl::core::Tower& tower) { return &tower.aero; });

    // core/mooring.h
    py::class_<seahowl::core::Mooring, std::shared_ptr<seahowl::core::Mooring>, seahowl::core::ComponentDynamic>(
        m_core, "Mooring")
        .def(py::init<std::shared_ptr<seahowl::elasto::MooringElastoFEA>,
                      std::shared_ptr<seahowl::hydro::MooringHydro>>())
        .def("set_length", &seahowl::core::Mooring::set_length)
        .def("set_diameter", &seahowl::core::Mooring::set_diameter)
        .def("set_discretization_elasto", &seahowl::core::Mooring::set_discretization_elasto)
        .def("set_discretization_hydro", &seahowl::core::Mooring::set_discretization_hydro)
        .def("update_positions_hydro", &seahowl::core::Mooring::update_positions_hydro)
        .def("update_loads_elasto", &seahowl::core::Mooring::update_loads_elasto)
        .def_property_readonly("elasto", [](seahowl::core::Mooring& mooring) { return &mooring.elasto; })
        .def_property_readonly("hydro", [](seahowl::core::Mooring& mooring) { return &mooring.hydro; });
    py::class_<seahowl::core::MooringSystem, std::shared_ptr<seahowl::core::MooringSystem>,
               seahowl::core::ComponentDynamic>(m_core, "MooringSystem")
        .def_readonly("moorings", &seahowl::core::MooringSystem::moorings)
        .def("add_mooring", &seahowl::core::MooringSystem::add_mooring)
        .def_property_readonly("elasto", [](seahowl::core::MooringSystem& ms) { return &ms.elasto; })
        .def_property_readonly("hydro", [](seahowl::core::MooringSystem& ms) { return &ms.hydro; });

    // core/foundation.h
    py::class_<seahowl::core::Foundation, std::shared_ptr<seahowl::core::Foundation>, seahowl::core::ComponentDynamic>(
        m_core, "Foundation");

    // core/floater.h
    py::class_<seahowl::core::Floater, std::shared_ptr<seahowl::core::Floater>, seahowl::core::Foundation>(m_core,
                                                                                                           "Floater")
        .def(py::init<std::shared_ptr<seahowl::elasto::FloaterElasto>, std::shared_ptr<seahowl::hydro::FloaterHydro>>())
        .def_property_readonly("elasto", [](seahowl::core::Floater& floater) { return &floater.elasto; })
        .def_property_readonly("hydro", [](seahowl::core::Floater& floater) { return &floater.hydro; })
        .def_property_readonly(
            "mooring_system", [](seahowl::core::Floater& floater) { return floater.mooring_system.get(); },
            py::return_value_policy::reference_internal);

    // core/monopile.h
    py::class_<seahowl::core::Monopile, std::shared_ptr<seahowl::core::Monopile>, seahowl::core::Tower,
               seahowl::core::Foundation>(m_core, "Monopile", pybind11::multiple_inheritance())
        .def(py::init<std::shared_ptr<seahowl::elasto::MonopileElasto>,
                      std::shared_ptr<seahowl::hydro::MonopileHydro>>())
        .def_property_readonly("elasto", [](seahowl::core::Monopile& monopile) { return &monopile.elasto; })
        .def_property_readonly("hydro", [](seahowl::core::Monopile& monopile) { return &monopile.hydro; });

    // core/rotor.h
    py::class_<seahowl::core::RotorNacelleAssembly, std::shared_ptr<seahowl::core::RotorNacelleAssembly>,
               seahowl::core::ComponentDynamic>(m_core, "RotorNacelleAssembly")
        .def(py::init<std::shared_ptr<seahowl::elasto::RotorNacelleAssemblyElasto>,
                      std::shared_ptr<seahowl::aero::RotorNacelleAssemblyAero>>())
        .def("update_positions_aero", &seahowl::core::RotorNacelleAssembly::update_positions_aero)
        .def("get_yaw_error", &seahowl::core::RotorNacelleAssembly::get_yaw_error)
        .def_readonly("rotor", &seahowl::core::RotorNacelleAssembly::rotor)
        .def_property_readonly("elasto", [](seahowl::core::RotorNacelleAssembly& rna) { return &rna.elasto; })
        .def_property_readonly("aero", [](seahowl::core::RotorNacelleAssembly& rna) { return &rna.aero; });

    py::class_<seahowl::core::Rotor, std::shared_ptr<seahowl::core::Rotor>, seahowl::core::ComponentDynamic>(m_core,
                                                                                                             "Rotor")
        .def(py::init<std::shared_ptr<seahowl::elasto::RotorElasto>, std::shared_ptr<seahowl::aero::RotorAero>>())
        .def_readonly("blades", &seahowl::core::Rotor::blades)
        .def_property_readonly("elasto", [](seahowl::core::Rotor& rotor) { return &rotor.elasto; })
        .def_property_readonly("aero", [](seahowl::core::Rotor& rotor) { return &rotor.aero; });

    // core/blade.h
    py::class_<seahowl::core::Blade, std::shared_ptr<seahowl::core::Blade>, seahowl::core::ComponentDynamic>(m_core,
                                                                                                             "Blade")
        .def(py::init<std::shared_ptr<seahowl::elasto::BladeElasto>, std::shared_ptr<seahowl::aero::BladeAero>>())
        .def("apply_pitch_increment", &seahowl::core::Blade::apply_pitch_increment)
        .def("set_discretization_elasto", &seahowl::core::Blade::set_discretization_elasto)
        .def("set_discretization_aero", &seahowl::core::Blade::set_discretization_aero)
        .def("update_positions_aero", &seahowl::core::Blade::update_positions_aero)
        .def("update_loads_elasto", &seahowl::core::Blade::update_loads_elasto)
        .def_property_readonly("elasto", [](seahowl::core::Blade& blade) { return &blade.elasto; })
        .def_property_readonly("aero", [](seahowl::core::Blade& blade) { return &blade.aero; });

    // core/system.h
    py::class_<seahowl::core::System, std::shared_ptr<seahowl::core::System>, seahowl::core::ComponentDynamic>(m_core,
                                                                                                               "System")
        .def(py::init<std::shared_ptr<seahowl::elasto::SystemElasto>, std::shared_ptr<seahowl::fluid::SystemFluid>>())
        .def("step", &seahowl::core::System::step)
        .def("get_time", &seahowl::core::System::get_time)
        .def("set_time", &seahowl::core::System::set_time)
        .def("run_presimulation", &seahowl::core::System::run_presimulation)
        .def("add", static_cast<void (seahowl::core::System::*)(std::shared_ptr<seahowl::core::Turbine> component)>(
                        &seahowl::core::System::add))
        .def("add",
             static_cast<void (seahowl::core::System::*)(std::shared_ptr<seahowl::core::ComponentDynamic> component)>(
                 &seahowl::core::System::add))
        .def_readonly("turbines", &seahowl::core::System::turbines)
        .def_readonly("components", &seahowl::core::System::components)
        .def_readwrite("env_model", &seahowl::core::System::env_model)
        .def_property_readonly("elasto", [](seahowl::core::System& system) { return &system.elasto; })
        .def_property_readonly("fluid", [](seahowl::core::System& system) { return &system.fluid; })
        .def_property_readonly("aero", [](seahowl::core::System& system) {
            seahowl::log("System.aero is deprecated, use System.fluid instead for aero/hydro access.", "warn");
            return &system.fluid;
        });
}
