#include <pybind11/pybind11.h>
#include <pybind11/stl.h>
#include <pybind11/eigen.h>

#include <seahowl/elasto/entities_elasto.h>
#include <seahowl/elasto/component_elasto.h>
#include <seahowl/elasto/reference_point_elasto.h>
#include <seahowl/elasto/blade_elasto.h>
#include <seahowl/elasto/rotor_elasto.h>
#include <seahowl/elasto/tower_elasto.h>
#include <seahowl/elasto/mooring_elasto.h>
#include <seahowl/elasto/floater_elasto.h>
#include <seahowl/elasto/turbine_elasto.h>
#include <seahowl/elasto/turbine_floating_elasto.h>
#include <seahowl/elasto/system_elasto.h>
#include <seahowl/elasto/chrono_adapters.h>

namespace py = pybind11;

void initialize_pyseahowl_elasto(py::module& m) {
    // submodule
    auto m_elasto = m.def_submodule("elasto", "Elasto submodule.");

    // elasto/entities_elasto.h
    py::class_<seahowl::elasto::EntityLoadable, std::shared_ptr<seahowl::elasto::EntityLoadable>,
               seahowl::EntityDynamic>(m_elasto, "EntityLoadable", pybind11::multiple_inheritance())
        .def("reset_loads", &seahowl::elasto::EntityLoadable::reset_loads)
        .def("set_force", &seahowl::elasto::EntityLoadable::set_force)
        .def("set_force", &seahowl::elasto::EntityLoadable::set_force)
        .def("get_force", &seahowl::elasto::EntityLoadable::get_force)
        .def("set_torque", &seahowl::elasto::EntityLoadable::set_torque)
        .def("get_torque", &seahowl::elasto::EntityLoadable::get_torque)
        .def("accumulate_force", &seahowl::elasto::EntityLoadable::accumulate_force)
        .def("accumulate_torque", &seahowl::elasto::EntityLoadable::accumulate_torque);
    py::class_<seahowl::elasto::BodyElasto, std::shared_ptr<seahowl::elasto::BodyElasto>,
               seahowl::elasto::EntityLoadable>(m_elasto, "BodyElasto", pybind11::multiple_inheritance())
        .def("set_mass", &seahowl::elasto::BodyElasto::set_mass)
        .def("get_mass", &seahowl::elasto::BodyElasto::get_mass)
        .def("set_inertia_diagonal", &seahowl::elasto::BodyElasto::set_inertia_diagonal)
        .def("set_inertia_matrix", &seahowl::elasto::BodyElasto::set_inertia_matrix)
        .def("get_inertia_matrix", &seahowl::elasto::BodyElasto::get_inertia_matrix)
        .def("set_fixed", &seahowl::elasto::BodyElasto::set_fixed);
    py::class_<seahowl::elasto::NodeElasto, std::shared_ptr<seahowl::elasto::NodeElasto>,
               seahowl::elasto::EntityLoadable>(m_elasto, "NodeElasto", pybind11::multiple_inheritance())
        .def("get_direction", &seahowl::elasto::NodeElasto::get_direction)
        .def("set_fixed", &seahowl::elasto::NodeElasto::set_fixed);
    py::class_<seahowl::elasto::ElementElasto, std::shared_ptr<seahowl::elasto::ElementElasto>>(m_elasto,
                                                                                                "ElementElasto")
        .def("set_nodes", &seahowl::elasto::ElementElasto::set_nodes)
        .def("evaluate_position_rotation", &seahowl::elasto::ElementElasto::evaluate_position_rotation)
        .def("evaluate_force_torque", &seahowl::elasto::ElementElasto::evaluate_force_torque)
        .def("get_force", &seahowl::elasto::ElementElasto::get_force)
        .def("get_torque", &seahowl::elasto::ElementElasto::get_torque)
        .def("get_mass", &seahowl::elasto::ElementElasto::get_mass);
    py::class_<seahowl::elasto::ElementBladeElasto, std::shared_ptr<seahowl::elasto::ElementBladeElasto>,
               seahowl::elasto::ElementElasto>(m_elasto, "ElementBladeElasto")
        .def("set_prebend", &seahowl::elasto::ElementBladeElasto::set_prebend);
    py::class_<seahowl::elasto::ElementMooringElasto, std::shared_ptr<seahowl::elasto::ElementMooringElasto>,
               seahowl::elasto::ElementElasto>(m_elasto, "ElementMooringElasto")
        .def("set_properties", &seahowl::elasto::ElementMooringElasto::set_properties)
        .def("set_rest_length", &seahowl::elasto::ElementMooringElasto::set_rest_length)
        .def("get_rest_length", &seahowl::elasto::ElementMooringElasto::get_rest_length);
    py::class_<seahowl::elasto::Link, std::shared_ptr<seahowl::elasto::Link>>(m_elasto, "Link")
        .def("initialize", [](seahowl::elasto::Link& link, seahowl::elasto::BodyElasto& body,
                              seahowl::elasto::BodyElasto& body2) { link.initialize(body, body2); })
        .def("initialize", [](seahowl::elasto::Link& link, seahowl::elasto::BodyElasto& body,
                              seahowl::elasto::NodeElasto& node) { link.initialize(body, node); })
        .def("initialize", [](seahowl::elasto::Link& link, seahowl::elasto::NodeElasto& node,
                              seahowl::elasto::BodyElasto& body) { link.initialize(node, body); })
        .def("set_constraints", &seahowl::elasto::Link::set_constraints)
        .def("get_reaction_force", &seahowl::elasto::Link::get_reaction_force)
        .def("get_reaction_torque", &seahowl::elasto::Link::get_reaction_torque);
    py::class_<seahowl::elasto::MeshElasto, std::shared_ptr<seahowl::elasto::MeshElasto>>(m_elasto, "MeshElasto");
    py::class_<seahowl::elasto::SystemElasto, std::shared_ptr<seahowl::elasto::SystemElasto>>(m_elasto, "SystemElasto")
        .def("add", [](seahowl::elasto::SystemElasto& system, seahowl::elasto::BodyElasto& body) { system.add(body); })
        .def("add", [](seahowl::elasto::SystemElasto& system, seahowl::elasto::Link& link) { system.add(link); })
        .def("step", &seahowl::elasto::SystemElasto::step)
        .def("get_time", &seahowl::elasto::SystemElasto::get_time)
        .def("set_gravitational_acceleration", &seahowl::elasto::SystemElasto::set_gravitational_acceleration)
        .def("get_gravitational_acceleration", &seahowl::elasto::SystemElasto::get_gravitational_acceleration)
        .def("do_statics", &seahowl::elasto::SystemElasto::do_statics);

    // elasto/chrono_adapters.h
    py::class_<seahowl::elasto::BodyElastoChrono, std::shared_ptr<seahowl::elasto::BodyElastoChrono>,
               seahowl::elasto::BodyElasto>(m_elasto, "BodyElastoChrono")
        .def(py::init<>());
    py::class_<seahowl::elasto::NodeElastoChrono, std::shared_ptr<seahowl::elasto::NodeElastoChrono>,
               seahowl::elasto::NodeElasto>(m_elasto, "NodeElastoChrono")
        .def(py::init<seahowl::Vector3d&, seahowl::Quaternion&>());
    py::class_<seahowl::elasto::NodeElastoChronoD, std::shared_ptr<seahowl::elasto::NodeElastoChronoD>,
               seahowl::elasto::NodeElasto>(m_elasto, "NodeElastoChronoD")
        .def(py::init<seahowl::Vector3d&, seahowl::Vector3d&>());
    py::class_<seahowl::elasto::ElementBladeElastoChrono, std::shared_ptr<seahowl::elasto::ElementBladeElastoChrono>,
               seahowl::elasto::ElementBladeElasto>(m_elasto, "ElementBladeElastoChrono")
        .def(py::init<>());
    py::class_<seahowl::elasto::ElementBladeElastoChronoFPM,
               std::shared_ptr<seahowl::elasto::ElementBladeElastoChronoFPM>, seahowl::elasto::ElementBladeElasto>(
        m_elasto, "ElementBladeElastoChronoFPM")
        .def(py::init<>());
    py::class_<seahowl::elasto::ElementMooringElastoChrono,
               std::shared_ptr<seahowl::elasto::ElementMooringElastoChrono>, seahowl::elasto::ElementMooringElasto>(
        m_elasto, "ElementMooringElastoChrono")
        .def(py::init<>());
    py::class_<seahowl::elasto::LinkChrono, std::shared_ptr<seahowl::elasto::LinkChrono>, seahowl::elasto::Link>(
        m_elasto, "LinkChrono")
        .def(py::init<>());
    py::class_<seahowl::elasto::LinkChronoCable, std::shared_ptr<seahowl::elasto::LinkChronoCable>,
               seahowl::elasto::Link>(m_elasto, "LinkChronoCable")
        .def(py::init<>());
    py::class_<seahowl::elasto::MeshElastoChrono, std::shared_ptr<seahowl::elasto::MeshElastoChrono>,
               seahowl::elasto::MeshElasto>(m_elasto, "MeshElastoChrono")
        .def(py::init<>());
    py::class_<seahowl::elasto::SystemElastoChrono, std::shared_ptr<seahowl::elasto::SystemElastoChrono>,
               seahowl::elasto::SystemElasto>(m_elasto, "SystemElastoChrono")
        .def(py::init<>());

    // elasto/reference_point_elasto.h
    py::class_<seahowl::elasto::ReferencePointElasto, std::shared_ptr<seahowl::elasto::ReferencePointElasto>>(
        m_elasto, "ReferencePointElasto")
        .def_readwrite("coordinates", &seahowl::elasto::ReferencePointElasto::coordinates)
        .def_readwrite("fraction", &seahowl::elasto::ReferencePointElasto::fraction)
        .def(py::init<>());
    py::class_<seahowl::elasto::BladeReferencePointElasto, std::shared_ptr<seahowl::elasto::BladeReferencePointElasto>,
               seahowl::elasto::ReferencePointElasto>(m_elasto, "BladeReferencePointElasto")
        .def_readwrite("offset_elastic", &seahowl::elasto::BladeReferencePointElasto::offset_elastic)
        .def_readwrite("offset_gravity", &seahowl::elasto::BladeReferencePointElasto::offset_gravity)
        .def_readwrite("stiffness_matrix", &seahowl::elasto::BladeReferencePointElasto::stiffness_matrix)
        .def_readwrite("mass_matrix", &seahowl::elasto::BladeReferencePointElasto::mass_matrix)
        .def_readwrite("structural_twist", &seahowl::elasto::BladeReferencePointElasto::structural_twist)
        .def_readwrite("damping_coefficients", &seahowl::elasto::BladeReferencePointElasto::damping_coefficients)
        .def(py::init<>());
    py::class_<seahowl::elasto::TowerReferencePointElasto, std::shared_ptr<seahowl::elasto::TowerReferencePointElasto>,
               seahowl::elasto::ReferencePointElasto>(m_elasto, "TowerReferencePointElasto")
        .def_readwrite("density", &seahowl::elasto::TowerReferencePointElasto::density)
        .def_readwrite("stiffness_axial", &seahowl::elasto::TowerReferencePointElasto::stiffness_axial)
        .def_readwrite("stiffness_foreaft", &seahowl::elasto::TowerReferencePointElasto::stiffness_foreaft)
        .def_readwrite("stiffness_sideside", &seahowl::elasto::TowerReferencePointElasto::stiffness_sideside)
        .def_readwrite("stiffness_torsion", &seahowl::elasto::TowerReferencePointElasto::stiffness_torsion)
        .def_readwrite("damping_coefficients", &seahowl::elasto::TowerReferencePointElasto::damping_coefficients)
        .def(py::init<>());

    // elasto/component_elasto.h
    py::class_<seahowl::elasto::ComponentElasto, std::shared_ptr<seahowl::elasto::ComponentElasto>>(m_elasto,
                                                                                                    "ComponentElasto")
        .def("assemble", &seahowl::elasto::ComponentElasto::assemble)
        .def("reset_loads", &seahowl::elasto::ComponentElasto::reset_loads)
        .def("rotate", &seahowl::elasto::ComponentElasto::rotate)
        .def("translate", &seahowl::elasto::ComponentElasto::translate)
        .def("get_mass", &seahowl::elasto::ComponentElasto::get_mass);
    py::class_<seahowl::elasto::ComponentElastoFEA, std::shared_ptr<seahowl::elasto::ComponentElastoFEA>,
               seahowl::elasto::ComponentElasto>(m_elasto, "ComponentElastoFEA")
        .def_readonly("nodes", &seahowl::elasto::ComponentElastoFEA::nodes)
        .def_readonly("elements", &seahowl::elasto::ComponentElastoFEA::elements)
        .def_readwrite("discretization_fractions", &seahowl::elasto::ComponentElastoFEA::discretization_fractions)
        .def("evaluate_position_rotation", &seahowl::elasto::ComponentElastoFEA::evaluate_position_rotation)
        .def("accumulate_element_load", &seahowl::elasto::ComponentElastoFEA::accumulate_element_load);

    // elasto/blade_elasto.h
    py::class_<seahowl::elasto::BladeElasto, std::shared_ptr<seahowl::elasto::BladeElasto>,
               seahowl::elasto::ComponentElasto>(m_elasto, "BladeElasto")
        .def_readonly("pitch", &seahowl::elasto::BladeElasto::pitch)
        .def_readonly("azimuth0", &seahowl::elasto::BladeElasto::azimuth0)
        .def_readonly("precone", &seahowl::elasto::BladeElasto::precone)
        .def_readwrite("reference_points", &seahowl::elasto::BladeElasto::reference_points)
        .def_readwrite("discretization_fractions", &seahowl::elasto::BladeElasto::discretization_fractions)
        .def("apply_pitch_increment", &seahowl::elasto::BladeElasto::apply_pitch_increment)
        .def("get_blade_root_moment", &seahowl::elasto::BladeElasto::get_blade_root_moment)
        .def("get_blade_root_force", &seahowl::elasto::BladeElasto::get_blade_root_force)
        .def("get_entity_along_blade", &seahowl::elasto::BladeElasto::get_entity_along_blade)
        .def("accumulate_load_along_blade", &seahowl::elasto::BladeElasto::accumulate_load_along_blade)
        .def("attach_root_to_body", &seahowl::elasto::BladeElasto::attach_root_to_body);
    py::class_<seahowl::elasto::BladeElastoFEA, std::shared_ptr<seahowl::elasto::BladeElastoFEA>,
               seahowl::elasto::BladeElasto, seahowl::elasto::ComponentElastoFEA>(m_elasto, "BladeElastoFEA")
        .def_readonly("discretized_points", &seahowl::elasto::BladeElastoFEA::discretized_points)
        .def_readwrite("fpm_mode", &seahowl::elasto::BladeElastoFEA::fpm_mode)
        .def(py::init<>());
    py::class_<seahowl::elasto::BladeElastoRigid, std::shared_ptr<seahowl::elasto::BladeElastoRigid>,
               seahowl::elasto::BladeElasto>(m_elasto, "BladeElastoRigid")
        .def(py::init<>());

    // elasto/rotor_elasto.h
    py::class_<seahowl::elasto::RotorElasto, std::shared_ptr<seahowl::elasto::RotorElasto>>(m_elasto, "RotorElasto")
        .def("apply_collective_pitch_increment", &seahowl::elasto::RotorElasto::apply_collective_pitch_increment)
        .def_readonly("blades", &seahowl::elasto::RotorElasto::blades)
        .def_property_readonly("body_hub", [](seahowl::elasto::RotorElasto& rotor) { return rotor.body_hub.get(); })
        .def_readonly("pitch_collective", &seahowl::elasto::RotorElasto::pitch_collective);
    py::class_<seahowl::elasto::RotorNacelleAssemblyElasto,
               std::shared_ptr<seahowl::elasto::RotorNacelleAssemblyElasto>, seahowl::elasto::ComponentElasto>(
        m_elasto, "RotorNacelleAssemblyElasto")
        .def(py::init<>())
        .def("get_rpm", &seahowl::elasto::RotorNacelleAssemblyElasto::get_rpm)
        .def("get_axial_thrust", &seahowl::elasto::RotorNacelleAssemblyElasto::get_axial_thrust)
        .def("get_axial_torque", &seahowl::elasto::RotorNacelleAssemblyElasto::get_axial_torque)
        .def("get_azimuth", &seahowl::elasto::RotorNacelleAssemblyElasto::get_azimuth)
        .def_property_readonly(
            "rotor", [](seahowl::elasto::RotorNacelleAssemblyElasto& rna) { return rna.rotor.get(); },
            py::return_value_policy::reference_internal)
        .def_property_readonly(
            "body_shaft", [](seahowl::elasto::RotorNacelleAssemblyElasto& rna) { return rna.body_shaft.get(); },
            py::return_value_policy::reference_internal)
        .def_property_readonly(
            "body_bedplate", [](seahowl::elasto::RotorNacelleAssemblyElasto& rna) { return rna.body_bedplate.get(); },
            py::return_value_policy::reference_internal)
        .def_property_readonly(
            "body_nacelle", [](seahowl::elasto::RotorNacelleAssemblyElasto& rna) { return rna.body_nacelle.get(); },
            py::return_value_policy::reference_internal)
        .def_property_readonly(
            "body_yaw_bearing",
            [](seahowl::elasto::RotorNacelleAssemblyElasto& rna) { return rna.body_yaw_bearing.get(); },
            py::return_value_policy::reference_internal)
        .def_property_readonly(
            "link_shaft_hub", [](seahowl::elasto::RotorNacelleAssemblyElasto& rna) { return rna.link_shaft_hub.get(); },
            py::return_value_policy::reference_internal)
        .def_property_readonly(
            "link_shaft_bedplate",
            [](seahowl::elasto::RotorNacelleAssemblyElasto& rna) { return rna.link_shaft_bedplate.get(); },
            py::return_value_policy::reference_internal)
        .def_property_readonly(
            "link_shaft_nacelle",
            [](seahowl::elasto::RotorNacelleAssemblyElasto& rna) { return rna.link_bedplate_nacelle.get(); },
            py::return_value_policy::reference_internal)
        .def_property_readonly(
            "link_nacelle_yaw_bearing",
            [](seahowl::elasto::RotorNacelleAssemblyElasto& rna) { return rna.link_nacelle_yaw_bearing.get(); },
            py::return_value_policy::reference_internal)
        .def_property_readonly(
            "link_nacelle_yaw_bearing",
            [](seahowl::elasto::RotorNacelleAssemblyElasto& rna) { return rna.link_towertop_yaw_bearing.get(); },
            py::return_value_policy::reference_internal);

    // elasto/tower_elasto.h
    py::class_<seahowl::elasto::TowerElasto, std::shared_ptr<seahowl::elasto::TowerElasto>,
               seahowl::elasto::ComponentElastoFEA>(m_elasto, "TowerElasto")
        .def_readwrite("reference_points", &seahowl::elasto::TowerElasto::reference_points)
        .def_readonly("discretized_points", &seahowl::elasto::TowerElasto::discretized_points)
        .def(py::init<>())
        .def("build", &seahowl::elasto::TowerElasto::build)
        .def("get_tower_base_moment", &seahowl::elasto::TowerElasto::get_tower_base_moment);

    // elasto/turbine_elasto.h
    py::class_<seahowl::elasto::TurbineElasto, std::shared_ptr<seahowl::elasto::TurbineElasto>,
               seahowl::elasto::ComponentElasto>(m_elasto, "TurbineElasto")
        .def_readonly("rna", &seahowl::elasto::TurbineElasto::rna)
        .def_readonly("tower", &seahowl::elasto::TurbineElasto::tower)
        .def(py::init<>());

    // elasto/floater_elasto.h
    py::class_<seahowl::elasto::FloaterElasto, std::shared_ptr<seahowl::elasto::FloaterElasto>,
               seahowl::elasto::ComponentElasto>(m_elasto, "FloaterElasto")
        .def("add_body", &seahowl::elasto::FloaterElasto::add_body)
        .def("get_body", &seahowl::elasto::FloaterElasto::get_body, py::return_value_policy::reference_internal)
        .def("add_fairlead", &seahowl::elasto::FloaterElasto::add_fairlead)
        .def("get_fairlead_link", &seahowl::elasto::FloaterElasto::get_fairlead_link,
             py::return_value_policy::reference_internal)
        .def("get_fairlead_body", &seahowl::elasto::FloaterElasto::get_fairlead_body,
             py::return_value_policy::reference_internal)
        .def("set_tower_connection_body_name", &seahowl::elasto::FloaterElasto::set_tower_connection_body_name)
        .def("get_tower_connection_body", &seahowl::elasto::FloaterElasto::get_tower_connection_body,
             py::return_value_policy::reference_internal);

    // elasto/mooring_elasto.h
    py::class_<seahowl::elasto::MooringSystem, std::shared_ptr<seahowl::elasto::MooringSystem>,
               seahowl::elasto::ComponentElasto>(m_elasto, "MooringSystem")
        .def(py::init<>())
        .def_readwrite("moorings", &seahowl::elasto::MooringSystem::moorings)
        .def_readwrite("anchors", &seahowl::elasto::MooringSystem::anchors);
    py::class_<seahowl::elasto::MooringElasto, std::shared_ptr<seahowl::elasto::MooringElasto>,
               seahowl::elasto::ComponentElasto>(m_elasto, "MooringElasto")
        .def_property_readonly("fairlead", [](seahowl::elasto::MooringElasto& mooring) { return &mooring.fairlead; })
        .def_property_readonly("anchor", [](seahowl::elasto::MooringElasto& mooring) { return &mooring.anchor; })
        .def("get_tension_fairlead", &seahowl::elasto::MooringElasto::get_tension_fairlead)
        .def("get_tension_anchor", &seahowl::elasto::MooringElasto::get_tension_anchor);
    py::class_<seahowl::elasto::MooringElastoFEA, std::shared_ptr<seahowl::elasto::MooringElastoFEA>,
               seahowl::elasto::MooringElasto, seahowl::elasto::ComponentElastoFEA>(m_elasto, "MooringElastoFEA")
        .def(py::init<seahowl::elasto::BodyElasto&, seahowl::elasto::BodyElasto&>());

    // elasto/turbine_floating_elasto.h
    py::class_<seahowl::elasto::TurbineFloatingElasto, std::shared_ptr<seahowl::elasto::TurbineFloatingElasto>,
               seahowl::elasto::TurbineElasto>(m_elasto, "TurbineFloatingElasto")
        //.def_property_readonly(
        //    "floater", [](seahowl::elasto::TurbineFloatingElasto& turbine) { return turbine.floater.get(); },
        //    py::return_value_policy::reference_internal)
        .def_readwrite("floater", &seahowl::elasto::TurbineFloatingElasto::floater)
        .def_property_readonly(
            "link_floater_tower",
            [](seahowl::elasto::TurbineFloatingElasto& turbine) { return turbine.link_floater_tower.get(); },
            py::return_value_policy::reference_internal)
        .def_property_readonly(
            "mooring_system",
            [](seahowl::elasto::TurbineFloatingElasto& turbine) { return turbine.mooring_system.get(); },
            py::return_value_policy::reference_internal)
        .def(py::init<>());
}
