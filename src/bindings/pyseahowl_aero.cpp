#include <pybind11/pybind11.h>
#include <pybind11/stl.h>
#include <pybind11/eigen.h>

#include <seahowl/aero/system_aero.h>
#include <seahowl/aero/blade_aero.h>
#include <seahowl/aero/rotor_aero.h>
#include <seahowl/aero/turbine_aero.h>
#include <seahowl/env/wind_models.h>

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
        .def_readwrite("blades", &seahowl::aero::RotorNacelleAssemblyAero::blades)
        .def_readonly("body_hub", &seahowl::aero::RotorNacelleAssemblyAero::body_hub)
        .def_readonly("body_nacelle", &seahowl::aero::RotorNacelleAssemblyAero::body_nacelle);

    // aero/turbine_aero.h
    py::class_<seahowl::aero::TurbineAero, std::shared_ptr<seahowl::aero::TurbineAero>>(m_aero, "TurbineAero")
        .def(py::init<>())
        .def_readonly("rna", &seahowl::aero::TurbineAero::rna);

    // env/wind_models.h
    py::class_<seahowl::env::WindModel, std::shared_ptr<seahowl::env::WindModel>>(m_aero, "WindModel")
        .def("get_fluid_velocity", &seahowl::env::WindModel::get_fluid_velocity)
        .def("get_fluid_density", &seahowl::env::WindModel::get_fluid_density);
    py::class_<seahowl::env::ShearedWind, std::shared_ptr<seahowl::env::ShearedWind>, seahowl::env::WindModel>(
        m_aero, "ShearedWind")
        .def_readwrite("shear_coefficient", &seahowl::env::ShearedWind::shear_coefficient)
        .def_readwrite("reference_height", &seahowl::env::ShearedWind::reference_height)
        .def_readwrite("reference_length", &seahowl::env::ShearedWind::reference_length)
        .def_readwrite("direction_gravity", &seahowl::env::ShearedWind::direction_gravity);
    py::class_<seahowl::env::ConstantWind, std::shared_ptr<seahowl::env::ConstantWind>, seahowl::env::ShearedWind>(
        m_aero, "ConstantWind")
        .def(py::init<>())
        .def("set_wind_velocity", &seahowl::env::ConstantWind::set_wind_velocity)
        .def_readwrite("shear_coefficient", &seahowl::env::ConstantWind::shear_coefficient);
    py::class_<seahowl::env::WindRamp, std::shared_ptr<seahowl::env::WindRamp>, seahowl::env::ShearedWind>(m_aero,
                                                                                                           "WindRamp")
        .def(py::init<>())
        .def("set_wind_ramp", &seahowl::env::WindRamp::set_wind_ramp)
        .def_readwrite("time_start", &seahowl::env::WindRamp::time_start)
        .def_readwrite("time_end", &seahowl::env::WindRamp::time_end)
        .def_readwrite("wind_velocity_start", &seahowl::env::WindRamp::wind_velocity_start)
        .def_readwrite("wind_velocity_end", &seahowl::env::WindRamp::wind_velocity_end);
}
