#include <pybind11/pybind11.h>
#include <pybind11/stl.h>
#include <pybind11/eigen.h>

#include <seahowl/core/component.h>
#include <seahowl/core/simulation.h>
#include <seahowl/core/turbine.h>
#include <seahowl/core/turbine_floating.h>
#include <seahowl/elasto/turbine_elasto.h>
#include <seahowl/aero/turbine_aero.h>
#include <seahowl/core/rotor.h>
#include <seahowl/core/blade.h>
#include <seahowl/elasto/blade_elasto.h>
#include <seahowl/core/tower.h>
#include <seahowl/elasto/tower_elasto.h>
#include <seahowl/core/system.h>
#include <seahowl/elasto/system_elasto.h>
#include <seahowl/aero/system_aero.h>
#include <seahowl/env/wind_models.h>
#include <seahowl/servo/controller.h>
#include <seahowl/io/read_json.h>
#include <seahowl/io/read_json.h>

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
        .def("apply_fluid_model", &seahowl::core::ComponentDynamic::apply_fluid_model)
        .def("apply_soil_model", &seahowl::core::ComponentDynamic::apply_soil_model);

    // core/simulation.h
    py::class_<seahowl::core::Simulation, std::shared_ptr<seahowl::core::Simulation>>(m_core, "Simulation")
        .def(py::init<>())
        .def("run_all", &seahowl::core::Simulation::run_all)
        .def("step", &seahowl::core::Simulation::step)
        .def("populate_from_file", &seahowl::core::Simulation::populate_from_file)
        .def("initialize", &seahowl::core::Simulation::initialize)
        .def("initialize_from_file", &seahowl::core::Simulation::initialize_from_file)
        .def_readwrite("dt", &seahowl::core::Simulation::dt)
        .def_readwrite("dt_output", &seahowl::core::Simulation::dt_output)
        .def_readwrite("duration", &seahowl::core::Simulation::duration)
        .def_property_readonly(
            "system_core", [](seahowl::core::Simulation& sim) { return sim.system_core.get(); },
            py::return_value_policy::reference_internal)
        .def_property_readonly(
            "outputs", [](seahowl::core::Simulation& sim) { return sim.outputs.get(); },
            py::return_value_policy::reference_internal);

    // core/turbine.h
    py::class_<seahowl::core::Turbine, std::shared_ptr<seahowl::core::Turbine>, seahowl::core::ComponentDynamic>(
        m_core, "Turbine")
        .def(py::init<seahowl::elasto::TurbineElasto&, seahowl::aero::TurbineAero&>())
        .def("get_generated_power", &seahowl::core::Turbine::get_generated_power)
        .def("get_generator_rpm", &seahowl::core::Turbine::get_generator_rpm)
        .def_property_readonly("elasto", [](seahowl::core::Turbine& turbine) { return &turbine.elasto; })
        .def_property_readonly("aero", [](seahowl::core::Turbine& turbine) { return &turbine.aero; })
        .def_readonly("controller", &seahowl::core::Turbine::controller)
        .def_readonly("tower", &seahowl::core::Turbine::tower)
        .def_readonly("rna", &seahowl::core::Turbine::rna);

    // core/turbine_floating.h
    py::class_<seahowl::core::TurbineFloating, std::shared_ptr<seahowl::core::TurbineFloating>, seahowl::core::Turbine>(
        m_core, "TurbineFloating")
        .def(py::init<seahowl::elasto::TurbineFloatingElasto&, seahowl::aero::TurbineAero&>())
        .def_property_readonly("elasto", [](seahowl::core::Turbine& turbine) { return &turbine.elasto; })
        .def_property_readonly("aero", [](seahowl::core::Turbine& turbine) { return &turbine.aero; });
    m_core.def(
        "get_turbine_floating_reference",
        [](seahowl::core::Turbine& turbine) {
            try {
                return dynamic_cast<seahowl::core::TurbineFloating&>(turbine);
            } catch (const std::bad_cast& e) {
                throw std::runtime_error("Cannot convert Turbine reference to TurbineFloating reference.");
            }
        },
        py::return_value_policy::reference_internal);

    // core/tower.h
    py::class_<seahowl::core::Tower, std::shared_ptr<seahowl::core::Tower>, seahowl::core::ComponentDynamic>(m_core,
                                                                                                             "Tower")
        .def_property_readonly("elasto", [](seahowl::core::Tower& tower) { return &tower.elasto; });

    // core/rotor.h
    py::class_<seahowl::core::RotorNacelleAssembly, std::shared_ptr<seahowl::core::RotorNacelleAssembly>,
               seahowl::core::ComponentDynamic>(m_core, "RotorNacelleAssembly")
        .def_readonly("blades", &seahowl::core::RotorNacelleAssembly::blades)
        .def_property_readonly("elasto", [](seahowl::core::RotorNacelleAssembly& rna) { return &rna.elasto; })
        .def_property_readonly("aero", [](seahowl::core::RotorNacelleAssembly& rna) { return &rna.aero; });

    // core/blade.h
    py::class_<seahowl::core::Blade, std::shared_ptr<seahowl::core::Blade>, seahowl::core::ComponentDynamic>(m_core,
                                                                                                             "Blade")
        .def("set_discretization_elasto", &seahowl::core::Blade::set_discretization_elasto)
        .def("set_discretization_aero", &seahowl::core::Blade::set_discretization_aero)
        .def_property_readonly("elasto", [](seahowl::core::Blade& blade) { return &blade.elasto; });

    // core/system.h
    py::class_<seahowl::core::System, std::shared_ptr<seahowl::core::System>, seahowl::core::ComponentDynamic>(m_core,
                                                                                                               "System")
        .def(py::init<seahowl::elasto::SystemElasto&, seahowl::aero::SystemAero&>())
        .def("step", &seahowl::core::System::step)
        .def("get_time", &seahowl::core::System::get_time)
        .def("set_time", &seahowl::core::System::set_time)
        .def("run_presetup", &seahowl::core::System::run_presetup)
        .def("run_presimulation", &seahowl::core::System::run_presimulation)
        .def("add_turbine", &seahowl::core::System::add_turbine)
        .def_readonly("turbines", &seahowl::core::System::turbines)
        .def_readonly("components", &seahowl::core::System::components)
        .def_readwrite("fluid_model", &seahowl::core::System::fluid_model)
        .def_property_readonly("elasto", [](seahowl::core::System& system) { return &system.elasto; })
        .def_property_readonly("aero", [](seahowl::core::System& system) { return &system.aero; });
}
