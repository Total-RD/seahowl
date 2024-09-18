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
        .def("set_constraints", &seahowl::elasto::Link::set_constraints)
        .def("get_reaction_force", &seahowl::elasto::Link::get_reaction_force)
        .def("get_reaction_torque", &seahowl::elasto::Link::get_reaction_torque)
        .def("initialize", &seahowl::elasto::Link::initialize);
    py::class_<seahowl::elasto::SpringLinear, std::shared_ptr<seahowl::elasto::SpringLinear>>(m_elasto, "SpringLinear")
        .def("set_rest_length", &seahowl::elasto::SpringLinear::set_rest_length)
        .def("set_spring_coefficient", &seahowl::elasto::SpringLinear::set_spring_coefficient)
        .def("set_damping_coefficient", &seahowl::elasto::SpringLinear::set_damping_coefficient)
        .def("get_force", &seahowl::elasto::SpringLinear::get_force)
        .def("initialize", &seahowl::elasto::SpringLinear::initialize)
        .def("initialize_with_anchors", &seahowl::elasto::SpringLinear::initialize_with_anchors);
    py::class_<seahowl::elasto::LinkMatrixStiffnessDamping,
               std::shared_ptr<seahowl::elasto::LinkMatrixStiffnessDamping>>(m_elasto, "LinkMatrixStiffnessDamping")
        .def("initialize", &seahowl::elasto::LinkMatrixStiffnessDamping::initialize)
        .def("set_stiffness_matrix", &seahowl::elasto::LinkMatrixStiffnessDamping::set_stiffness_matrix)
        .def("set_damping_matrix", &seahowl::elasto::LinkMatrixStiffnessDamping::set_damping_matrix);
    py::class_<seahowl::elasto::MeshElasto, std::shared_ptr<seahowl::elasto::MeshElasto>>(m_elasto, "MeshElasto");
    py::class_<seahowl::elasto::SystemElasto, std::shared_ptr<seahowl::elasto::SystemElasto>>(m_elasto, "SystemElasto")
        .def("step", &seahowl::elasto::SystemElasto::step)
        .def("assemble", &seahowl::elasto::SystemElasto::assemble)
        .def("get_time", &seahowl::elasto::SystemElasto::get_time)
        .def("set_gravitational_acceleration", &seahowl::elasto::SystemElasto::set_gravitational_acceleration)
        .def("get_gravitational_acceleration", &seahowl::elasto::SystemElasto::get_gravitational_acceleration)
        .def("do_statics", &seahowl::elasto::SystemElasto::do_statics)
        .def("add", static_cast<void (seahowl::elasto::SystemElasto::*)(seahowl::elasto::BodyElasto & body)>(
                        &seahowl::elasto::SystemElasto::add))
        .def("add", static_cast<void (seahowl::elasto::SystemElasto::*)(seahowl::elasto::MeshElasto & mesh)>(
                        &seahowl::elasto::SystemElasto::add))
        .def("add", static_cast<void (seahowl::elasto::SystemElasto::*)(seahowl::elasto::Link & link)>(
                        &seahowl::elasto::SystemElasto::add))
        .def("add",
             static_cast<void (seahowl::elasto::SystemElasto::*)(seahowl::elasto::LinkMatrixStiffnessDamping & link)>(
                 &seahowl::elasto::SystemElasto::add))
        .def("add", static_cast<void (seahowl::elasto::SystemElasto::*)(seahowl::elasto::SpringLinear & spring)>(
                        &seahowl::elasto::SystemElasto::add))
        .def("add", static_cast<void (seahowl::elasto::SystemElasto::*)(
                        std::shared_ptr<seahowl::elasto::TurbineElasto> turbine)>(&seahowl::elasto::SystemElasto::add))
        .def("add",
             static_cast<void (seahowl::elasto::SystemElasto::*)(
                 std::shared_ptr<seahowl::elasto::ComponentElasto> component)>(&seahowl::elasto::SystemElasto::add))
        .def_readonly("turbines", &seahowl::elasto::SystemElasto::turbines)
        .def_readonly("components", &seahowl::elasto::SystemElasto::components);

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
        .def(py::init<>())
        .def("set_properties", &seahowl::elasto::ElementMooringElasto::set_properties)
        .def("set_rest_length", &seahowl::elasto::ElementMooringElastoChrono::set_rest_length)
        .def("get_rest_length", &seahowl::elasto::ElementMooringElastoChrono::get_rest_length);
    py::class_<seahowl::elasto::LinkChrono, std::shared_ptr<seahowl::elasto::LinkChrono>, seahowl::elasto::Link>(
        m_elasto, "LinkChrono")
        .def(py::init<>());

    py::class_<seahowl::elasto::SpringLinearChrono, std::shared_ptr<seahowl::elasto::SpringLinearChrono>,
               seahowl::elasto::SpringLinear>(m_elasto, "SpringLinearChrono")
        .def(py::init<>());
    py::class_<seahowl::elasto::LinkMatrixStiffnessDampingChrono,
               std::shared_ptr<seahowl::elasto::LinkMatrixStiffnessDampingChrono>,
               seahowl::elasto::LinkMatrixStiffnessDamping>(m_elasto, "LinkMatrixStiffnessDampingChrono")
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
        .def_readwrite("stiffness_foreaft_shear", &seahowl::elasto::TowerReferencePointElasto::stiffness_foreaft_shear)
        .def_readwrite("stiffness_sideside_shear",
                       &seahowl::elasto::TowerReferencePointElasto::stiffness_sideside_shear)
        .def_readwrite("inertia_foreaft", &seahowl::elasto::TowerReferencePointElasto::inertia_foreaft)
        .def_readwrite("inertia_sideside", &seahowl::elasto::TowerReferencePointElasto::inertia_sideside)
        .def_readwrite("damping_coefficients", &seahowl::elasto::TowerReferencePointElasto::damping_coefficients)
        .def(py::init<>())
        .def("set_properties_cylinder", &seahowl::elasto::TowerReferencePointElasto::set_properties_cylinder);

    // elasto/component_elasto.h
    py::class_<seahowl::elasto::ComponentElasto, std::shared_ptr<seahowl::elasto::ComponentElasto>>(m_elasto,
                                                                                                    "ComponentElasto")

        .def("build", &seahowl::elasto::ComponentElasto::build)
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
        .def("apply_blade_pitch_increment", &seahowl::elasto::RotorElasto::apply_blade_pitch_increment)
        .def_readonly("blades", &seahowl::elasto::RotorElasto::blades)
        .def_property_readonly("body_hub", [](seahowl::elasto::RotorElasto& rotor) { return rotor.body_hub.get(); })
        .def_readonly("pitch_collective", &seahowl::elasto::RotorElasto::pitch_collective);
    py::class_<seahowl::elasto::RotorNacelleAssemblyElasto,
               std::shared_ptr<seahowl::elasto::RotorNacelleAssemblyElasto>, seahowl::elasto::ComponentElasto>(
        m_elasto, "RotorNacelleAssemblyElasto")
        .def(py::init<>())
        .def("get_rpm", &seahowl::elasto::RotorNacelleAssemblyElasto::get_rpm)
        .def("accumulate_electrical_torque", &seahowl::elasto::RotorNacelleAssemblyElasto::accumulate_electrical_torque)
        .def("get_axial_thrust", &seahowl::elasto::RotorNacelleAssemblyElasto::get_axial_thrust)
        .def("get_electrical_torque", &seahowl::elasto::RotorNacelleAssemblyElasto::get_electrical_torque)
        .def("get_axial_torque", &seahowl::elasto::RotorNacelleAssemblyElasto::get_axial_torque)
        .def("get_azimuth", &seahowl::elasto::RotorNacelleAssemblyElasto::get_azimuth)
        .def_property_readonly(
            "rotor", [](seahowl::elasto::RotorNacelleAssemblyElasto& rna) { return rna.rotor.get(); },
            py::return_value_policy::reference_internal)
        .def_property_readonly(
            "body_shaft", [](seahowl::elasto::RotorNacelleAssemblyElasto& rna) { return rna.body_shaft.get(); },
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
            "link_shaft_nacelle",
            [](seahowl::elasto::RotorNacelleAssemblyElasto& rna) { return rna.link_shaft_nacelle.get(); },
            py::return_value_policy::reference_internal)
        .def_property_readonly(
            "link_shaft_yaw_bearing",
            [](seahowl::elasto::RotorNacelleAssemblyElasto& rna) { return rna.link_shaft_yaw_bearing.get(); },
            py::return_value_policy::reference_internal)
        .def_property_readonly(
            "link_shaft_yaw_bearing",
            [](seahowl::elasto::RotorNacelleAssemblyElasto& rna) { return rna.link_towertop_yaw_bearing.get(); },
            py::return_value_policy::reference_internal);

    // elasto/tower_elasto.h
    py::class_<seahowl::elasto::TowerElasto, std::shared_ptr<seahowl::elasto::TowerElasto>,
               seahowl::elasto::ComponentElastoFEA>(m_elasto, "TowerElasto")
        .def_readwrite("reference_points", &seahowl::elasto::TowerElasto::reference_points)
        .def_readonly("discretized_points", &seahowl::elasto::TowerElasto::discretized_points)
        .def(py::init<>())
        .def("get_tower_base_moment", &seahowl::elasto::TowerElasto::get_tower_base_moment);

    // elasto/turbine_elasto.h
    py::class_<seahowl::elasto::TurbineElasto, std::shared_ptr<seahowl::elasto::TurbineElasto>,
               seahowl::elasto::ComponentElasto>(m_elasto, "TurbineElasto")
        .def_readonly("rna", &seahowl::elasto::TurbineElasto::rna)
        .def_readonly("tower", &seahowl::elasto::TurbineElasto::tower)
        .def_readonly("foundation", &seahowl::elasto::TurbineElasto::foundation)
        .def(py::init<>());

    // elasto/floater_elasto.h
    py::class_<seahowl::elasto::FoundationElasto, std::shared_ptr<seahowl::elasto::FoundationElasto>,
               seahowl::elasto::ComponentElasto>(m_elasto, "FoundationElasto")
        .def("link_to_entity", &seahowl::elasto::FoundationElasto::link_to_entity);

    py::class_<seahowl::elasto::FloaterElasto, std::shared_ptr<seahowl::elasto::FloaterElasto>,
               seahowl::elasto::FoundationElasto>(m_elasto, "FloaterElasto")
        .def_property_readonly(
            "mooring_system", [](seahowl::elasto::FloaterElasto& floater) { return floater.mooring_system.get(); },
            py::return_value_policy::reference_internal)
        .def_property_readonly(
            "body_main", [](seahowl::elasto::FloaterElasto& floater) { return floater.body_main.get(); },
            py::return_value_policy::reference_internal)
        .def_readwrite("damping_matrix", &seahowl::elasto::FloaterElasto::damping_matrix)
        .def("add_body", &seahowl::elasto::FloaterElasto::add_body)
        .def("get_body", &seahowl::elasto::FloaterElasto::get_body, py::return_value_policy::reference_internal)
        .def("add_fairlead", &seahowl::elasto::FloaterElasto::add_fairlead)
        .def("get_fairlead_link", &seahowl::elasto::FloaterElasto::get_fairlead_link,
             py::return_value_policy::reference_internal)
        .def("get_fairlead_body", &seahowl::elasto::FloaterElasto::get_fairlead_body,
             py::return_value_policy::reference_internal);

    // elasto/mooring_elasto.h
    py::class_<seahowl::elasto::MooringSystemElasto, std::shared_ptr<seahowl::elasto::MooringSystemElasto>,
               seahowl::elasto::ComponentElasto>(m_elasto, "MooringSystemElasto")
        .def(py::init<>())
        .def_readonly("moorings", &seahowl::elasto::MooringSystemElasto::moorings)
        .def_readwrite("anchors", &seahowl::elasto::MooringSystemElasto::anchors)
        .def("add_mooring", &seahowl::elasto::MooringSystemElasto::add_mooring);
    py::class_<seahowl::elasto::MooringElasto, std::shared_ptr<seahowl::elasto::MooringElasto>,
               seahowl::elasto::ComponentElasto>(m_elasto, "MooringElasto")
        .def_property_readonly("fairlead", [](seahowl::elasto::MooringElasto& mooring) { return &mooring.fairlead; })
        .def_property_readonly("anchor", [](seahowl::elasto::MooringElasto& mooring) { return &mooring.anchor; })
        .def("get_tension_fairlead", &seahowl::elasto::MooringElasto::get_tension_fairlead)
        .def("get_tension_anchor", &seahowl::elasto::MooringElasto::get_tension_anchor)
        .def("get_length", &seahowl::elasto::MooringElasto::get_length)
        .def("set_length", &seahowl::elasto::MooringElasto::set_length)
        .def("set_diameter", &seahowl::elasto::MooringElasto::set_diameter);
    py::class_<seahowl::elasto::MooringElastoFEA, std::shared_ptr<seahowl::elasto::MooringElastoFEA>,
               seahowl::elasto::MooringElasto, seahowl::elasto::ComponentElastoFEA>(m_elasto, "MooringElastoFEA")
        .def(py::init<seahowl::elasto::BodyElasto&, seahowl::elasto::BodyElasto&>())
        .def("build", &seahowl::elasto::MooringElastoFEA::build)
        .def_readwrite("length", &seahowl::elasto::MooringElastoFEA::length)
        .def_readwrite("diameter", &seahowl::elasto::MooringElastoFEA::diameter)
        .def_readwrite("stiffness_axial", &seahowl::elasto::MooringElastoFEA::stiffness_axial)
        .def_readwrite("stiffness_bending", &seahowl::elasto::MooringElastoFEA::stiffness_bending)
        .def_readwrite("density_linear", &seahowl::elasto::MooringElastoFEA::density_linear)
        .def_property_readonly(
            "fairlead_link", [](seahowl::elasto::MooringElastoFEA& mooring) { return mooring.fairlead_link.get(); },
            py::return_value_policy::reference_internal)
        .def_property_readonly(
            "anchor_link", [](seahowl::elasto::MooringElastoFEA& mooring) { return mooring.anchor_link.get(); },
            py::return_value_policy::reference_internal)
        .def_property_readonly("fairlead", [](seahowl::elasto::MooringElastoFEA& mooring) { return &mooring.fairlead; })

        .def_property_readonly("anchor", [](seahowl::elasto::MooringElastoFEA& mooring) { return &mooring.anchor; });
}
