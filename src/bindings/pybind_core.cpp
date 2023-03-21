#include <pybind11/pybind11.h>
#include <pybind11/stl.h>
#include <pybind11/eigen.h>

#include <seahowl/commons/entities.h>

#include <seahowl/elasto/entities_elasto.h>
#include <seahowl/elasto/component_elasto.h>
#include <seahowl/elasto/blade_elasto.h>
#include <seahowl/elasto/rotor_elasto.h>
#include <seahowl/elasto/tower_elasto.h>
#include <seahowl/elasto/turbine_elasto.h>
#include <seahowl/elasto/chrono_adapters.h>

#include <seahowl/aero/system_aero.h>
#include <seahowl/aero/blade_aero.h>
#include <seahowl/aero/rotor_aero.h>
#include <seahowl/aero/turbine_aero.h>

#include <seahowl/servo/controller.h>

#include <seahowl/core/component.h>
#include <seahowl/core/turbine.h>
#include <seahowl/core/rotor.h>
#include <seahowl/core/blade.h>
#include <seahowl/core/tower.h>
#include <seahowl/core/system.h>
#include <seahowl/io/read_json.h>

namespace py = pybind11;

PYBIND11_MODULE(pyseahowl, m) {
    // io/read_json.h
    m.def("populate_blade_from_json", &populate_blade_from_json);
    m.def("populate_rotor_from_json", &populate_rotor_from_json);
    m.def("populate_turbine_from_json", &populate_turbine_from_json);
    m.def("populate_system_from_json", &populate_system_from_json);

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

    // ELASTO
    //
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
        .def_readonly("nodes", &seahowl::elasto::TowerElasto::nodes);

    // elasto/blade_elasto.h
    py::class_<seahowl::elasto::BladeElasto, std::shared_ptr<seahowl::elasto::BladeElasto>,
               seahowl::elasto::ComponentElastoFEA>(m_elasto, "BladeElasto")
        .def(py::init<>())
        .def("apply_pitch_increment", &seahowl::elasto::BladeElasto::apply_pitch_increment)
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

    // AERO
    //
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
    py::class_<seahowl::aero::RotorAero, std::shared_ptr<seahowl::aero::RotorAero>>(m_aero, "RotorAero")
        .def(py::init<>())
        .def_readwrite("blades", &seahowl::aero::RotorAero::blades)
        .def_readonly("body_hub", &seahowl::aero::RotorAero::body_hub)
        .def_readonly("body_nacelle", &seahowl::aero::RotorAero::body_nacelle);

    // aero/turbine_aero.h
    py::class_<seahowl::aero::TurbineAero, std::shared_ptr<seahowl::aero::TurbineAero>>(m_aero, "TurbineAero")
        .def(py::init<>())
        .def_readonly("rotor", &seahowl::aero::TurbineAero::rotor);

    // SERVO
    //
    auto m_servo = m.def_submodule("servo", "Servo submodule.");
    // servo/controller.h
    py::class_<seahowl::servo::Controller, std::shared_ptr<seahowl::servo::Controller>>(m_servo, "Controller")
        .def_readwrite("has_pitch_control", &seahowl::servo::Controller::has_pitch_control)
        .def_readwrite("has_torque_control", &seahowl::servo::Controller::has_torque_control);

    // CORE
    //
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
        .def_readonly("rotor", &seahowl::core::Turbine::rotor);

    // core/tower.h
    py::class_<seahowl::core::Tower, std::shared_ptr<seahowl::core::Tower>, seahowl::core::ComponentDynamic>(m_core,
                                                                                                             "Tower")
        .def_property_readonly("elasto", [](seahowl::core::Tower& tower) { return &tower.elasto; });

    // core/rotor.h
    py::class_<seahowl::core::Rotor, std::shared_ptr<seahowl::core::Rotor>, seahowl::core::ComponentDynamic>(m_core,
                                                                                                             "Rotor")
        .def_property_readonly("elasto", [](seahowl::core::Rotor& rotor) { return &rotor.elasto; })
        .def_property_readonly("aero", [](seahowl::core::Rotor& rotor) { return &rotor.aero; });

    // core/blade.h
    py::class_<seahowl::core::Blade, std::shared_ptr<seahowl::core::Blade>, seahowl::core::ComponentDynamic>(m_core,
                                                                                                             "Blade")
        .def("set_discretization_elasto", &seahowl::core::Blade::set_discretization_elasto)
        .def("set_discretization_aero", &seahowl::core::Blade::set_discretization_aero)
        .def_property_readonly("elasto", [](seahowl::core::Blade& blade) { return &blade.elasto; });

    // core/system.h
    py::class_<seahowl::core::System, std::shared_ptr<seahowl::core::System>, seahowl::core::ComponentDynamic>(m_core,
                                                                                                               "System")
        .def(py::init<>())
        .def("step", &seahowl::core::System::step)
        .def_readonly("turbines", &seahowl::core::System::turbines)
        .def_readwrite("system_elasto", &seahowl::core::System::system_elasto)
        .def_readwrite("system_aero", &seahowl::core::System::system_aero);
}
