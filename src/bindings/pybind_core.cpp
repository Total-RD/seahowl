#include <pybind11/pybind11.h>
#include <pybind11/stl.h>

#include <seahowl/elasto/entities_elasto.h>
#include <seahowl/elasto/component_elasto.h>
#include <seahowl/elasto/blade_elasto.h>
#include <seahowl/elasto/rotor_elasto.h>
#include <seahowl/elasto/tower_elasto.h>
#include <seahowl/elasto/turbine_elasto.h>

#include <seahowl/core/component.h>
#include <seahowl/core/turbine.h>
#include <seahowl/core/rotor.h>
#include <seahowl/core/blade.h>
#include <seahowl/io/read_json.h>

namespace py = pybind11;

class PyComponentElasto : public seahowl::elasto::ComponentElasto {
  public:
    /* Inherit the constructors */
    using ComponentElasto::ComponentElasto;

    /* Trampoline (need one for each virtual function) */
    void rotate(double angle, const seahowl::Vector3d& axis) const override {
        PYBIND11_OVERRIDE_PURE(void, ComponentElasto, rotate, angle, axis);
    };
    void translate(const seahowl::Vector3d& translation_vector) const override {
        PYBIND11_OVERRIDE_PURE(void, ComponentElasto, translate, translation_vector);
    };
};

class PyComponentElastoFEA : public seahowl::elasto::ComponentElastoFEA {
  public:
    /* Inherit the constructors */
    using ComponentElastoFEA::ComponentElastoFEA;

    /* Trampoline (need one for each virtual function) */
    void rotate(double angle, const seahowl::Vector3d& axis) const override {
        PYBIND11_OVERRIDE_PURE(void, ComponentElastoFEA, rotate, angle, axis);
    };
    void translate(const seahowl::Vector3d& translation_vector) const override {
        PYBIND11_OVERRIDE_PURE(void, ComponentElastoFEA, translate, translation_vector);
    };
};

class PyComponentDynamic : public seahowl::core::ComponentDynamic {
  public:
    /* Inherit the constructors */
    using ComponentDynamic::ComponentDynamic;

    /* Trampoline (need one for each virtual function) */
    void initialize(double time, double dt) override {
        PYBIND11_OVERRIDE_PURE(void, ComponentDynamic, init, time, dt);
    };
    void prestep(double time, double dt) override {
        PYBIND11_OVERRIDE_PURE(void, ComponentDynamic, prestep, time, dt);
    };
    void poststep(double time, double dt) override {
        PYBIND11_OVERRIDE_PURE(void, ComponentDynamic, poststep, time, dt);
    };
};

PYBIND11_MODULE(pyseahowl, m) {
    // io/read_json.h
    m.def("populate_blade_from_json", &populate_blade_from_json);
    m.def("populate_rotor_from_json", &populate_rotor_from_json);
    m.def("populate_turbine_from_json", &populate_turbine_from_json);

    // ELASTO
    //
    auto elasto = m.def_submodule("elasto", "Elasto submodule.");
    // elasto/entities_elasto.h
    py::class_<seahowl::elasto::BodyElasto, std::shared_ptr<seahowl::elasto::BodyElasto>>(elasto, "BodyElasto")
        .def("set_fixed", &seahowl::elasto::BodyElasto::set_fixed);
    py::class_<seahowl::elasto::NodeElasto, std::shared_ptr<seahowl::elasto::NodeElasto>>(elasto, "NodeElasto")
        .def("set_fixed", &seahowl::elasto::NodeElasto::set_fixed);

    // elasto/component_elasto.h
    py::class_<seahowl::elasto::ComponentElasto, std::shared_ptr<seahowl::elasto::ComponentElasto>, PyComponentElasto>(
        elasto, "ComponentElasto")
        .def("rotate", &seahowl::elasto::ComponentElasto::rotate)
        .def("translate", &seahowl::elasto::ComponentElasto::translate);
    py::class_<seahowl::elasto::ComponentElastoFEA, std::shared_ptr<seahowl::elasto::ComponentElastoFEA>,
               PyComponentElastoFEA>(elasto, "ComponentElastoFEA")
        .def(py::init<>())
        .def("rotate", &seahowl::elasto::ComponentElastoFEA::rotate)
        .def("translate", &seahowl::elasto::ComponentElastoFEA::translate)
        .def_readonly("nodes", &seahowl::elasto::TowerElasto::nodes);

    // elasto/blade_elasto.h
    py::class_<seahowl::elasto::BladeElasto, std::shared_ptr<seahowl::elasto::BladeElasto>,
               seahowl::elasto::ComponentElastoFEA>(elasto, "BladeElasto")
        .def(py::init<>())
        .def("apply_pitch_increment", &seahowl::elasto::BladeElasto::apply_pitch_increment)
        .def_readonly("pitch", &seahowl::elasto::BladeElasto::pitch)
        .def_readonly("azimuth0", &seahowl::elasto::BladeElasto::azimuth0);

    // elasto/rotor_elasto.h
    py::class_<seahowl::elasto::RotorElasto, std::shared_ptr<seahowl::elasto::RotorElasto>,
               seahowl::elasto::ComponentElasto>(elasto, "RotorElasto")
        .def(py::init<>())
        .def("apply_collective_pitch_increment", &seahowl::elasto::RotorElasto::apply_collective_pitch_increment)
        .def("get_rpm", &seahowl::elasto::RotorElasto::get_rpm)
        .def("get_axial_thrust", &seahowl::elasto::RotorElasto::get_axial_thrust)
        .def("get_axial_torque", &seahowl::elasto::RotorElasto::get_axial_torque)
        .def("get_azimuth", &seahowl::elasto::RotorElasto::get_azimuth);

    // elasto/tower_elasto.h
    py::class_<seahowl::elasto::TowerElasto, std::shared_ptr<seahowl::elasto::TowerElasto>,
               seahowl::elasto::ComponentElastoFEA>(elasto, "TowerElasto")
        .def(py::init<>());

    // elasto/turbine_elasto.h
    py::class_<seahowl::elasto::TurbineElasto, std::shared_ptr<seahowl::elasto::TurbineElasto>>(elasto, "TurbineElasto")
        .def_readonly("rotor", &seahowl::elasto::TurbineElasto::rotor)
        .def_readonly("tower", &seahowl::elasto::TurbineElasto::tower)
        .def(py::init<>());

    // CORE
    //
    auto core = m.def_submodule("core", "Core submodule.");
    // core/utils.h
    py::class_<seahowl::core::ComponentDynamic, std::shared_ptr<seahowl::core::ComponentDynamic>, PyComponentDynamic>(
        core, "ComponentDynamic")
        .def("initialize", &seahowl::core::ComponentDynamic::initialize)
        .def("prestep", &seahowl::core::ComponentDynamic::prestep)
        .def("poststep", &seahowl::core::ComponentDynamic::poststep);

    // core/turbine.h
    py::class_<seahowl::core::Turbine, std::shared_ptr<seahowl::core::Turbine>, seahowl::core::ComponentDynamic>(
        core, "Turbine")
        .def("get_generated_power", &seahowl::core::Turbine::get_generated_power)
        .def("get_generator_rpm", &seahowl::core::Turbine::get_generator_rpm)
        .def("build", &seahowl::core::Turbine::build)
        .def_property_readonly("elasto", [](seahowl::core::Turbine& turbine) { return &turbine.elasto; });

    // core/rotor.h
    py::class_<seahowl::core::Rotor, std::shared_ptr<seahowl::core::Rotor>, seahowl::core::ComponentDynamic>(core,
                                                                                                             "Rotor")
        .def_property_readonly("elasto", [](seahowl::core::Rotor& rotor) { return &rotor.elasto; });

    // core/blade.h
    py::class_<seahowl::core::Blade, std::shared_ptr<seahowl::core::Blade>, seahowl::core::ComponentDynamic>(core,
                                                                                                             "Blade")
        .def("set_discretization_elasto", &seahowl::core::Blade::set_discretization_elasto)
        .def("set_discretization_aero", &seahowl::core::Blade::set_discretization_aero)
        .def_property_readonly("elasto", [](seahowl::core::Blade& blade) { return &blade.elasto; });
}
