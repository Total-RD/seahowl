#include <pybind11/pybind11.h>
#include <pybind11/stl.h>
#include <pybind11/eigen.h>

#include <seahowl/servo/controller.h>
#include <seahowl/servo/controller_discon.h>

namespace py = pybind11;

void initialize_pyseahowl_servo(py::module& m) {
    // submodule
    auto m_servo = m.def_submodule("servo", "Servo submodule.");

    // servo/controller.h
    py::class_<seahowl::servo::Controller, std::shared_ptr<seahowl::servo::Controller>>(m_servo, "Controller")
        .def_readwrite("has_pitch_control", &seahowl::servo::Controller::has_pitch_control)
        .def_readwrite("has_torque_control", &seahowl::servo::Controller::has_torque_control)
        .def("get_torque_elec", &seahowl::servo::Controller::get_torque_elec);
    py::class_<seahowl::servo::ControllerDISCON, std::shared_ptr<seahowl::servo::ControllerDISCON>,
               seahowl::servo::Controller>(m_servo, "ControllerDISCON");
}
