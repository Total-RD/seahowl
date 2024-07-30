#include <pybind11/pybind11.h>
#include <pybind11/stl.h>
#include <pybind11/eigen.h>

#include <seahowl/aero/system_aero.h>
#include <seahowl/aero/blade_aero.h>
#include <seahowl/aero/rotor_aero.h>
#include <seahowl/aero/tower_aero.h>
#include <seahowl/aero/turbine_aero.h>

namespace py = pybind11;

void initialize_pyseahowl_aero(py::module& m) {
    // submodule
    auto m_aero = m.def_submodule("aero", "Aero submodule.");

    // aero/system_aero.h
    py::class_<seahowl::aero::SystemAero, std::shared_ptr<seahowl::aero::SystemAero>>(m_aero, "SystemAero")
        .def(py::init<>());

    // aero/blade_aero.h
    py::class_<seahowl::aero::BladeNodeAero, std::shared_ptr<seahowl::aero::BladeNodeAero>,
               seahowl::EntityDynamicEigen>(m_aero, "BladeNodeAero")
        .def("get_offset_aero_absolute", &seahowl::aero::BladeNodeAero::get_offset_aero_absolute);
    py::class_<seahowl::aero::BladeElementAero, std::shared_ptr<seahowl::aero::BladeElementAero>>(m_aero,
                                                                                                  "BladeElementAero")
        .def("get_position", &seahowl::aero::BladeElementAero::get_position)
        .def("get_load", &seahowl::aero::BladeElementAero::get_load);
    py::class_<seahowl::aero::BladeAero, std::shared_ptr<seahowl::aero::BladeAero>>(m_aero, "BladeAero")
        .def(py::init<>())
        .def("get_total_load", &seahowl::aero::BladeAero::get_total_load)
        .def_readwrite("nodes", &seahowl::aero::BladeAero::nodes);

    // aero/rotor_aero.h
    py::class_<seahowl::aero::RotorNacelleAssemblyAero, std::shared_ptr<seahowl::aero::RotorNacelleAssemblyAero>>(
        m_aero, "RotorNacelleAssemblyAero")
        .def(py::init<>())
        .def_readonly("body_nacelle", &seahowl::aero::RotorNacelleAssemblyAero::body_nacelle)
        .def("compute_fluid_loads", &seahowl::aero::RotorNacelleAssemblyAero::compute_fluid_loads);

    // aero/rotor_aero.h
    py::class_<seahowl::aero::RotorAero, std::shared_ptr<seahowl::aero::RotorAero>>(m_aero, "RotorAero")
        .def_readwrite("blades", &seahowl::aero::RotorAero::blades)
        .def_readonly("body_hub", &seahowl::aero::RotorAero::body_hub)
        .def("compute_fluid_loads", &seahowl::aero::RotorAero::compute_fluid_loads);

    // aero/tower_aero.h
    py::class_<seahowl::aero::TowerAero, std::shared_ptr<seahowl::aero::TowerAero>>(m_aero, "TowerAero")
        .def(py::init<>())
        .def_readwrite("discretization_fractions", &seahowl::aero::TowerAero::discretization_fractions)
        .def_readwrite("nodes", &seahowl::aero::TowerAero::nodes)
        .def("build", &seahowl::aero::TowerAero::build)
        .def("compute_fluid_loads", &seahowl::aero::TowerAero::compute_fluid_loads);

    // aero/turbine_aero.h
    py::class_<seahowl::aero::TurbineAero, std::shared_ptr<seahowl::aero::TurbineAero>>(m_aero, "TurbineAero")
        .def(py::init<>())
        .def_readonly("rna", &seahowl::aero::TurbineAero::rna)
        .def("compute_fluid_loads", &seahowl::aero::TurbineAero::compute_fluid_loads);
}
