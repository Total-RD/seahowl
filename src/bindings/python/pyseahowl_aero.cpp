#include <pybind11/pybind11.h>
#include <pybind11/stl.h>
#include <pybind11/eigen.h>

#include <seahowl/commons/component_fluid.h>
#include <seahowl/aero/system_aero.h>
#include <seahowl/aero/blade_aero.h>
#include <seahowl/aero/rotor_aero.h>
#include <seahowl/aero/tower_aero.h>
#include <seahowl/aero/turbine_aero.h>
#include <seahowl/aero/reference_point_aero.h>

namespace py = pybind11;

void initialize_pyseahowl_aero(py::module& m) {
    // submodule
    auto m_aero = m.def_submodule("aero", "Aero submodule.");

    // aero/system_aero.h
    py::class_<seahowl::aero::SystemAero, std::shared_ptr<seahowl::aero::SystemAero>>(m_aero, "SystemAero")
        .def(py::init<>())
        .def("add",
             static_cast<void (seahowl::aero::SystemAero::*)(std::shared_ptr<seahowl::aero::TurbineAero> turbine)>(
                 &seahowl::aero::SystemAero::add))
        .def("add",
             static_cast<void (seahowl::aero::SystemAero::*)(std::shared_ptr<seahowl::ComponentFluid> component)>(
                 &seahowl::aero::SystemAero::add))
        .def_readonly("turbines", &seahowl::aero::SystemAero::turbines)
        .def_readonly("components", &seahowl::aero::SystemAero::components);
    ;

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
        .def("build", &seahowl::aero::BladeAero::build)
        .def_readwrite("nodes", &seahowl::aero::BladeAero::nodes);

    // aero/rotor_aero.h
    py::class_<seahowl::aero::RotorNacelleAssemblyAero, std::shared_ptr<seahowl::aero::RotorNacelleAssemblyAero>,
               seahowl::ComponentFluid>(m_aero, "RotorNacelleAssemblyAero")
        .def(py::init<>())
        .def_readonly("body_nacelle", &seahowl::aero::RotorNacelleAssemblyAero::body_nacelle);

    // aero/rotor_aero.h
    py::class_<seahowl::aero::RotorAero, std::shared_ptr<seahowl::aero::RotorAero>, seahowl::ComponentFluid>(
        m_aero, "RotorAero")
        .def_readwrite("blades", &seahowl::aero::RotorAero::blades)
        .def_readonly("body_hub", &seahowl::aero::RotorAero::body_hub);

    // aero/tower_aero.h
    py::class_<seahowl::aero::TowerAero, std::shared_ptr<seahowl::aero::TowerAero>, seahowl::ComponentFluid>(
        m_aero, "TowerAero")
        .def(py::init<>())
        .def_readwrite("discretization_fractions", &seahowl::aero::TowerAero::discretization_fractions)
        .def_readwrite("reference_points", &seahowl::aero::TowerAero::reference_points)
        .def_readwrite("nodes", &seahowl::aero::TowerAero::nodes);

    // aero/turbine_aero.h
    py::class_<seahowl::aero::TurbineAero, std::shared_ptr<seahowl::aero::TurbineAero>, seahowl::ComponentFluid>(
        m_aero, "TurbineAero")
        .def(py::init<>())
        .def_readonly("rna", &seahowl::aero::TurbineAero::rna)
        .def_readonly("tower", &seahowl::aero::TurbineAero::tower)
        .def_readonly("foundation", &seahowl::aero::TurbineAero::foundation);

    // aero/reference_point_aero.h
    py::class_<seahowl::aero::TowerReferencePointAero, std::shared_ptr<seahowl::aero::TowerReferencePointAero>>(
        m_aero, "TowerReferencePointAero")
        .def_readwrite("coordinates", &seahowl::aero::TowerReferencePointAero::coordinates)
        .def_readwrite("fraction", &seahowl::aero::TowerReferencePointAero::fraction)
        .def_readwrite("diameter", &seahowl::aero::TowerReferencePointAero::diameter)
        .def_readwrite("coefficients", &seahowl::aero::TowerReferencePointAero::coefficients)
        .def(py::init<>());
}
