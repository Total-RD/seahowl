#include <pybind11/pybind11.h>
#include <pybind11/stl.h>
#include <pybind11/eigen.h>

#include <seahowl/commons/entities.h>
#include <seahowl/io/read_json.h>
#include <seahowl/core/blade.h>
#include <seahowl/core/rotor.h>
#include <seahowl/core/turbine.h>
#include <seahowl/core/system.h>
#include <seahowl/elasto/blade_elasto.h>
#include <seahowl/elasto/tower_elasto.h>
#include <seahowl/elasto/rotor_elasto.h>
#include <seahowl/aero/blade_aero.h>
#include <seahowl/aero/tower_aero.h>
#include <seahowl/aero/rotor_aero.h>

namespace py = pybind11;

void initialize_pyseahowl_elasto(py::module& m);
void initialize_pyseahowl_aero(py::module& m);
void initialize_pyseahowl_servo(py::module& m);
void initialize_pyseahowl_core(py::module& m);

PYBIND11_MODULE(pyseahowl, m) {
    // commons.h
    py::class_<seahowl::Entity, std::shared_ptr<seahowl::Entity>>(m, "Entity")
        .def("get_position", &seahowl::Entity::get_position)
        .def("set_position", &seahowl::Entity::set_position)
        .def("get_rotation", &seahowl::Entity::get_rotation)
        .def("set_rotation", &seahowl::Entity::set_rotation);
    py::class_<seahowl::EntityDynamic, std::shared_ptr<seahowl::EntityDynamic>, seahowl::Entity>(m, "EntityDynamic")
        .def("get_velocity", &seahowl::EntityDynamic::get_velocity)
        .def("set_velocity", &seahowl::EntityDynamic::set_velocity)
        .def("get_acceleration", &seahowl::EntityDynamic::get_acceleration)
        .def("set_acceleration", &seahowl::EntityDynamic::set_acceleration)
        .def("get_rotational_velocity", &seahowl::EntityDynamic::get_rotational_velocity)
        .def("set_rotational_velocity", &seahowl::EntityDynamic::set_rotational_velocity)
        .def("get_rotational_acceleration", &seahowl::EntityDynamic::get_rotational_acceleration)
        .def("set_rotational_acceleration", &seahowl::EntityDynamic::set_rotational_acceleration);
    py::class_<seahowl::EntityEigen, std::shared_ptr<seahowl::EntityEigen>, seahowl::Entity>(m, "EntityEigen");
    py::class_<seahowl::EntityDynamicEigen, std::shared_ptr<seahowl::EntityDynamicEigen>, seahowl::EntityDynamic>(
        m, "EntityDynamicEigen");

    // elasto
    initialize_pyseahowl_elasto(m);

    // aero
    initialize_pyseahowl_aero(m);

    // servo
    initialize_pyseahowl_servo(m);

    // core
    initialize_pyseahowl_core(m);

    // io/read_json.h
    m.def("populate_blade_from_json", &populate_blade_from_json);
    m.def("populate_blade_elasto_from_json", &populate_blade_elasto_from_json);
    m.def("populate_blade_aero_from_json", &populate_blade_aero_from_json);
    m.def("populate_tower_from_json", &populate_tower_from_json);
    m.def("populate_tower_elasto_from_json", &populate_tower_elasto_from_json);
    m.def("populate_tower_aero_from_json", &populate_tower_aero_from_json);
    m.def("populate_rna_from_json", &populate_rna_from_json);
    m.def("populate_turbine_from_json", &populate_turbine_from_json);
    m.def("populate_system_from_json", &populate_system_from_json);
}
