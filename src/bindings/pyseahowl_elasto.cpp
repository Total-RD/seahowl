#include <pybind11/pybind11.h>
#include <pybind11/stl.h>
#include <pybind11/eigen.h>

#include <seahowl/elasto/entities_elasto.h>
#include <seahowl/elasto/component_elasto.h>
#include <seahowl/elasto/blade_elasto.h>
#include <seahowl/elasto/rotor_elasto.h>
#include <seahowl/elasto/tower_elasto.h>
#include <seahowl/elasto/turbine_elasto.h>
#include <seahowl/elasto/chrono_adapters.h>

namespace py = pybind11;

void initialize_pyseahowl_elasto(py::module& m) {
    // submodule
    auto m_elasto = m.def_submodule("elasto", "Elasto submodule.");

    // elasto/entities_elasto.h
    py::class_<seahowl::elasto::BodyElasto, std::shared_ptr<seahowl::elasto::BodyElasto>, seahowl::EntityDynamic>(
        m_elasto, "BodyElasto", pybind11::multiple_inheritance())
        .def("set_fixed", &seahowl::elasto::BodyElasto::set_fixed);
    py::class_<seahowl::elasto::NodeElasto, std::shared_ptr<seahowl::elasto::NodeElasto>, seahowl::EntityDynamic>(
        m_elasto, "NodeElasto", pybind11::multiple_inheritance())
        .def("get_load", &seahowl::elasto::NodeElasto::get_load)
        .def("get_torque", &seahowl::elasto::NodeElasto::get_torque)
        .def("set_fixed", &seahowl::elasto::NodeElasto::set_fixed);
    py::class_<seahowl::elasto::ElementElasto, std::shared_ptr<seahowl::elasto::ElementElasto>>(m_elasto,
                                                                                                "ElementElasto")
        .def("evaluate_position_rotation", &seahowl::elasto::ElementElasto::evaluate_position_rotation)
        .def("evaluate_force_torque", &seahowl::elasto::ElementElasto::evaluate_force_torque)
        .def("get_force", &seahowl::elasto::ElementElasto::get_force)
        .def("get_torque", &seahowl::elasto::ElementElasto::get_torque)
        .def("get_mass", &seahowl::elasto::ElementElasto::get_mass);
    py::class_<seahowl::elasto::ElementBladeElasto, std::shared_ptr<seahowl::elasto::ElementBladeElasto>,
               seahowl::elasto::ElementElasto>(m_elasto, "ElementBladeElasto");
    py::class_<seahowl::elasto::Link, std::shared_ptr<seahowl::elasto::Link>>(m_elasto, "Link")
        .def("get_reaction_force", &seahowl::elasto::Link::get_reaction_force)
        .def("get_reaction_torque", &seahowl::elasto::Link::get_reaction_torque);
    py::class_<seahowl::elasto::MeshElasto, std::shared_ptr<seahowl::elasto::MeshElasto>>(m_elasto, "MeshElasto");
    py::class_<seahowl::elasto::SystemElasto, std::shared_ptr<seahowl::elasto::SystemElasto>>(m_elasto, "SystemElasto")
        .def("get_time", &seahowl::elasto::SystemElasto::get_time)
        .def("do_statics", &seahowl::elasto::SystemElasto::do_statics);

    // elasto/chrono_adapters.h
    py::class_<seahowl::elasto::MeshElastoChrono, std::shared_ptr<seahowl::elasto::MeshElastoChrono>,
               seahowl::elasto::MeshElasto>(m_elasto, "MeshElastoChrono")
        .def(py::init<>());
    py::class_<seahowl::elasto::SystemElastoChrono, std::shared_ptr<seahowl::elasto::SystemElastoChrono>,
               seahowl::elasto::SystemElasto>(m_elasto, "SystemElastoChrono")
        .def(py::init<>());

    // elasto/component_elasto.h
    py::class_<seahowl::elasto::ComponentElasto, std::shared_ptr<seahowl::elasto::ComponentElasto>>(m_elasto,
                                                                                                    "ComponentElasto")
        .def("rotate", &seahowl::elasto::ComponentElasto::rotate)
        .def("translate", &seahowl::elasto::ComponentElasto::translate);
    py::class_<seahowl::elasto::ComponentElastoFEA, std::shared_ptr<seahowl::elasto::ComponentElastoFEA>,
               seahowl::elasto::ComponentElasto>(m_elasto, "ComponentElastoFEA")
        .def_readonly("nodes", &seahowl::elasto::ComponentElastoFEA::nodes)
        .def_readonly("elements", &seahowl::elasto::ComponentElastoFEA::elements);

    // elasto/blade_elasto.h
    py::class_<seahowl::elasto::BladeElasto, std::shared_ptr<seahowl::elasto::BladeElasto>,
               seahowl::elasto::ComponentElastoFEA>(m_elasto, "BladeElasto")
        .def(py::init<>())
        .def("apply_pitch_increment", &seahowl::elasto::BladeElasto::apply_pitch_increment)
        .def("get_blade_root_moment", &seahowl::elasto::BladeElasto::get_blade_root_moment)
        .def_readonly("pitch", &seahowl::elasto::BladeElasto::pitch)
        .def_readonly("azimuth0", &seahowl::elasto::BladeElasto::azimuth0);

    // elasto/rotor_elasto.h
    py::class_<seahowl::elasto::RotorElasto, std::shared_ptr<seahowl::elasto::RotorElasto>,
               seahowl::elasto::ComponentElasto>(m_elasto, "RotorElasto")
        .def(py::init<>())
        .def("apply_collective_pitch_increment", &seahowl::elasto::RotorElasto::apply_collective_pitch_increment)
        .def("get_rpm", &seahowl::elasto::RotorElasto::get_rpm)
        .def("get_axial_thrust", &seahowl::elasto::RotorElasto::get_axial_thrust)
        .def("get_axial_torque", &seahowl::elasto::RotorElasto::get_axial_torque)
        .def("get_azimuth", &seahowl::elasto::RotorElasto::get_azimuth)
        .def_readonly("blades", &seahowl::elasto::RotorElasto::blades)
        .def_property_readonly("body_hub", [](seahowl::elasto::RotorElasto& rotor) { return rotor.body_hub.get(); })
        .def_property_readonly(
            "body_shaft", [](seahowl::elasto::RotorElasto& rotor) { return rotor.body_shaft.get(); },
            py::return_value_policy::reference_internal)
        .def_property_readonly(
            "body_nacelle", [](seahowl::elasto::RotorElasto& rotor) { return rotor.body_nacelle.get(); },
            py::return_value_policy::reference_internal)
        .def_property_readonly(
            "body_yaw_bearing", [](seahowl::elasto::RotorElasto& rotor) { return rotor.body_yaw_bearing.get(); },
            py::return_value_policy::reference_internal)
        .def_property_readonly(
            "link_shaft_hub", [](seahowl::elasto::RotorElasto& rotor) { return rotor.link_shaft_hub.get(); },
            py::return_value_policy::reference_internal)
        .def_property_readonly(
            "link_shaft_nacelle", [](seahowl::elasto::RotorElasto& rotor) { return rotor.link_shaft_nacelle.get(); },
            py::return_value_policy::reference_internal)
        .def_property_readonly(
            "link_shaft_yaw_bearing",
            [](seahowl::elasto::RotorElasto& rotor) { return rotor.link_shaft_yaw_bearing.get(); },
            py::return_value_policy::reference_internal)
        .def_property_readonly(
            "link_shaft_yaw_bearing",
            [](seahowl::elasto::RotorElasto& rotor) { return rotor.link_towertop_yaw_bearing.get(); },
            py::return_value_policy::reference_internal)
        .def_readonly("pitch_collective", &seahowl::elasto::RotorElasto::pitch_collective);

    // elasto/tower_elasto.h
    py::class_<seahowl::elasto::TowerElasto, std::shared_ptr<seahowl::elasto::TowerElasto>,
               seahowl::elasto::ComponentElastoFEA>(m_elasto, "TowerElasto")
        .def(py::init<>());

    // elasto/turbine_elasto.h
    py::class_<seahowl::elasto::TurbineElasto, std::shared_ptr<seahowl::elasto::TurbineElasto>>(m_elasto,
                                                                                                "TurbineElasto")
        .def_readonly("rotor", &seahowl::elasto::TurbineElasto::rotor)
        .def_readonly("tower", &seahowl::elasto::TurbineElasto::tower)
        .def(py::init<>());
}
