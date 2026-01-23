#include <pybind11/pybind11.h>
#include <pybind11/stl.h>
#include <pybind11/eigen.h>

#include <seahowl/commons/numerics.h>
#include <seahowl/commons/entities.h>
#include <seahowl/commons/utils.h>
#include <seahowl/commons/component_fluid.h>

namespace py = pybind11;

void initialize_pyseahowl_env(py::module& m);
void initialize_pyseahowl_elasto(py::module& m);
void initialize_pyseahowl_aero(py::module& m);
void initialize_pyseahowl_hydro(py::module& m);
void initialize_pyseahowl_servo(py::module& m);
void initialize_pyseahowl_core(py::module& m);
void initialize_pyseahowl_io(py::module& m);

PYBIND11_MODULE(seahowl, m) {
    // utils.h
    m.def("set_log_level_global", &seahowl::set_log_level_global);
    m.def("log", &seahowl::log);

    // numerics.h - Quaternion binding
    py::class_<seahowl::Quaternion>(m, "Quaternion")
        .def(py::init<>())
        .def(py::init<double, double, double, double>(), py::arg("w"), py::arg("x"), py::arg("y"), py::arg("z"))
        .def(py::init([](const Eigen::Vector4d& coeffs) {
            return seahowl::Quaternion(coeffs(0), coeffs(1), coeffs(2), coeffs(3));  // w, x, y, z
        }))
        .def_property_readonly("w", [](const seahowl::Quaternion& q) { return q.w(); })
        .def_property_readonly("x", [](const seahowl::Quaternion& q) { return q.x(); })
        .def_property_readonly("y", [](const seahowl::Quaternion& q) { return q.y(); })
        .def_property_readonly("z", [](const seahowl::Quaternion& q) { return q.z(); })
        .def("coeffs", [](const seahowl::Quaternion& q) { return q.coeffs(); })
        .def("normalized", &seahowl::Quaternion::normalized)
        .def("inverse", &seahowl::Quaternion::inverse)
        .def("conjugate", &seahowl::Quaternion::conjugate)
        .def("norm", &seahowl::Quaternion::norm)
        .def("squaredNorm", &seahowl::Quaternion::squaredNorm)
        .def("toRotationMatrix", &seahowl::Quaternion::toRotationMatrix)
        .def("__mul__", [](const seahowl::Quaternion& q1, const seahowl::Quaternion& q2) { return q1 * q2; })
        .def("__repr__", [](const seahowl::Quaternion& q) {
            return "Quaternion(w=" + std::to_string(q.w()) + ", x=" + std::to_string(q.x()) +
                   ", y=" + std::to_string(q.y()) + ", z=" + std::to_string(q.z()) + ")";
        });

    // entities.h
    py::class_<seahowl::Entity, std::shared_ptr<seahowl::Entity>>(m, "Entity")
        .def("get_position", &seahowl::Entity::get_position)
        .def("set_position", &seahowl::Entity::set_position)
        .def("get_rotation", &seahowl::Entity::get_rotation)
        .def("set_rotation", &seahowl::Entity::set_rotation)
        .def("get_direction", &seahowl::Entity::get_direction)
        .def("get_rpy_angles", &seahowl::Entity::get_rpy_angles)
        .def("get_rotation_matrix", &seahowl::Entity::get_rotation_matrix)
        .def("set_rotation_matrix", &seahowl::Entity::set_rotation_matrix)
        .def("rotate", &seahowl::Entity::rotate)
        .def("translate", &seahowl::Entity::translate);
    py::class_<seahowl::EntityDynamic, std::shared_ptr<seahowl::EntityDynamic>, seahowl::Entity>(m, "EntityDynamic")
        .def("get_velocity", &seahowl::EntityDynamic::get_velocity)
        .def("set_velocity", &seahowl::EntityDynamic::set_velocity)
        .def("get_acceleration", &seahowl::EntityDynamic::get_acceleration)
        .def("set_acceleration", &seahowl::EntityDynamic::set_acceleration)
        .def("get_rotational_velocity", &seahowl::EntityDynamic::get_rotational_velocity)
        .def("set_rotational_velocity", &seahowl::EntityDynamic::set_rotational_velocity)
        .def("get_rotational_acceleration", &seahowl::EntityDynamic::get_rotational_acceleration)
        .def("set_rotational_acceleration", &seahowl::EntityDynamic::set_rotational_acceleration);
    py::class_<seahowl::EntityDynamicEigen, std::shared_ptr<seahowl::EntityDynamicEigen>, seahowl::EntityDynamic>(
        m, "EntityDynamicEigen")
        .def(py::init<>());

    // component
    py::class_<seahowl::ComponentFluid, std::shared_ptr<seahowl::ComponentFluid>>(m, "ComponentFluid")
        .def("build", &seahowl::ComponentFluid::build)
        .def("compute_env_loads", &seahowl::ComponentFluid::compute_env_loads)
        .def("setup_environment", &seahowl::ComponentFluid::setup_environment);

    // env
    initialize_pyseahowl_env(m);

    // elasto
    initialize_pyseahowl_elasto(m);

    // aero
    initialize_pyseahowl_aero(m);

    // aero
    initialize_pyseahowl_hydro(m);

    // servo
    initialize_pyseahowl_servo(m);

    // core
    initialize_pyseahowl_core(m);

    // io
    initialize_pyseahowl_io(m);
}
