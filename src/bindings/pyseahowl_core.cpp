#include <pybind11/pybind11.h>
#include <pybind11/stl.h>
#include <pybind11/eigen.h>

#include <seahowl/core/component.h>
#include <seahowl/core/turbine.h>
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

namespace py = pybind11;

void initialize_pyseahowl_core(py::module& m) {
    // submodule
    auto m_core = m.def_submodule("core", "Core submodule.");

    // core/utils.h
    py::class_<seahowl::core::ComponentDynamic, std::shared_ptr<seahowl::core::ComponentDynamic>>(m_core,
                                                                                                  "ComponentDynamic")
        .def("initialize", &seahowl::core::ComponentDynamic::initialize)
        .def("prestep", &seahowl::core::ComponentDynamic::prestep)
        .def("poststep", &seahowl::core::ComponentDynamic::poststep);

    // core/turbine.h
    py::class_<seahowl::core::Turbine, std::shared_ptr<seahowl::core::Turbine>, seahowl::core::ComponentDynamic>(
        m_core, "Turbine")
        .def(py::init<seahowl::elasto::TurbineElasto&, seahowl::aero::TurbineAero&>())
        .def("get_generated_power", &seahowl::core::Turbine::get_generated_power)
        .def("get_generator_rpm", &seahowl::core::Turbine::get_generator_rpm)
        .def("build", &seahowl::core::Turbine::build)
        .def_property_readonly("elasto", [](seahowl::core::Turbine& turbine) { return &turbine.elasto; })
        .def_property_readonly("aero", [](seahowl::core::Turbine& turbine) { return &turbine.aero; })
        .def_readonly("controller", &seahowl::core::Turbine::controller)
        .def_readonly("tower", &seahowl::core::Turbine::tower)
        .def_readonly("rna", &seahowl::core::Turbine::rna);

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
        .def_readonly("turbines", &seahowl::core::System::turbines)
        .def_readwrite("fluid_model", &seahowl::core::System::fluid_model)
        .def_property_readonly("elasto", [](seahowl::core::System& system) { return &system.elasto; })
        .def_property_readonly("aero", [](seahowl::core::System& system) { return &system.aero; });
}
