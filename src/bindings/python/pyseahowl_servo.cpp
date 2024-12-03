#include <pybind11/pybind11.h>
#include <pybind11/stl.h>
#include <pybind11/eigen.h>

#include <seahowl/servo/controller.h>
#include <seahowl/servo/controller_discon.h>
#include <seahowl/core/turbine.h>

namespace py = pybind11;

void initialize_pyseahowl_servo(py::module& m) {
    // submodule
    auto m_servo = m.def_submodule("servo", "Servo submodule.");

    // servo/controller.h
    py::class_<seahowl::servo::Controller, std::shared_ptr<seahowl::servo::Controller>>(m_servo, "Controller")
        .def_readwrite("has_pitch_control", &seahowl::servo::Controller::has_pitch_control)
        .def_readwrite("has_torque_control", &seahowl::servo::Controller::has_torque_control)
        .def_readwrite("has_yaw_control", &seahowl::servo::Controller::has_yaw_control)
        .def("initialize", &seahowl::servo::Controller::initialize)
        .def("step", &seahowl::servo::Controller::step)
        .def("poststep", &seahowl::servo::Controller::poststep)
        .def("get_torque_elec", &seahowl::servo::Controller::get_torque_elec)
        .def("get_pitch_blade", &seahowl::servo::Controller::get_pitch_blade)
        .def("get_collective_pitch", &seahowl::servo::Controller::get_collective_pitch);

    py::class_<seahowl::servo::ControllerVariableTorque, std::shared_ptr<seahowl::servo::ControllerVariableTorque>,
               seahowl::servo::Controller>(m_servo, "ControllerVariableTorque")
        .def(py::init<>())
        .def_readwrite("target_rpm", &seahowl::servo::ControllerVariableTorque::target_rpm);
    py::class_<seahowl::servo::ControllerDISCON, std::shared_ptr<seahowl::servo::ControllerDISCON>,
               seahowl::servo::Controller>(m_servo, "ControllerDISCON")
        .def(py::init<const std::string&, const std::string&>())
        .def("update_turbine_variables", &seahowl::servo::ControllerDISCON::update_turbine_variables);
}
