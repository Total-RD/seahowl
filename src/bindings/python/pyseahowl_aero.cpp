// SPDX-License-Identifier: Apache-2.0
// Copyright 2022–2026 TotalEnergies
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//

// pybind11 headers
#include <pybind11/eigen.h>
#include <pybind11/pybind11.h>
#include <pybind11/stl.h>

// SEAHOWL headers
#include <seahowl/fluid.h>

namespace py = pybind11;

void initialize_pyseahowl_aero(py::module& m_fluid) {
    // submodule
    auto m_aero = m_fluid.def_submodule("aero", "Aero submodule.");

    // aero/reference_point_aero.h
    py::class_<seahowl::aero::BladeReferencePointAero, std::shared_ptr<seahowl::aero::BladeReferencePointAero>>(
        m_aero, "BladeReferencePointAero")
        .def(py::init<>())
        .def_readwrite("fraction", &seahowl::aero::BladeReferencePointAero::fraction)
        .def_readwrite("coordinates", &seahowl::aero::BladeReferencePointAero::coordinates)
        .def_readwrite("offset_aero", &seahowl::aero::BladeReferencePointAero::offset_aero)
        .def_readwrite("chord", &seahowl::aero::BladeReferencePointAero::chord)
        .def_readwrite("structural_twist", &seahowl::aero::BladeReferencePointAero::structural_twist)
        .def_readwrite("airfoil_properties", &seahowl::aero::BladeReferencePointAero::airfoil_properties);

    // aero/blade_aero.h
    py::class_<seahowl::aero::BladeNodeAero, std::shared_ptr<seahowl::aero::BladeNodeAero>,
               seahowl::EntityDynamicEigen>(m_aero, "BladeNodeAero")
        .def(py::init<const seahowl::aero::BladeReferencePointAero&>())
        .def("get_offset_aero_absolute", &seahowl::aero::BladeNodeAero::get_offset_aero_absolute)
        .def_readwrite("load", &seahowl::aero::BladeNodeAero::load)
        .def_readwrite("moment", &seahowl::aero::BladeNodeAero::moment)
        .def_readwrite("wind_velocity", &seahowl::aero::BladeNodeAero::wind_velocity)
        .def_readwrite("wind_velocity_shadowed", &seahowl::aero::BladeNodeAero::wind_velocity_shadowed)
        .def_readwrite("relative_velocity_induced", &seahowl::aero::BladeNodeAero::relative_velocity_induced)
        .def_readwrite("properties", &seahowl::aero::BladeNodeAero::properties)
        .def_readwrite("chord_solidity", &seahowl::aero::BladeNodeAero::chord_solidity)
        .def_readwrite("induction_factor_axial", &seahowl::aero::BladeNodeAero::induction_factor_axial)
        .def_readwrite("induction_factor_tangential", &seahowl::aero::BladeNodeAero::induction_factor_tangential)
        .def_readwrite("radius", &seahowl::aero::BladeNodeAero::radius)
        .def_readwrite("distance_from_hub", &seahowl::aero::BladeNodeAero::distance_from_hub)
        .def_readwrite("distance_from_tip", &seahowl::aero::BladeNodeAero::distance_from_tip);
    py::class_<seahowl::aero::BladeElementAero, std::shared_ptr<seahowl::aero::BladeElementAero>>(m_aero,
                                                                                                  "BladeElementAero")
        .def("get_position", &seahowl::aero::BladeElementAero::get_position)
        .def("get_load", &seahowl::aero::BladeElementAero::get_load)
        .def("get_moment", &seahowl::aero::BladeElementAero::get_moment)
        .def("get_rotation", &seahowl::aero::BladeElementAero::get_rotation)
        .def("get_offset_aero_absolute", &seahowl::aero::BladeElementAero::get_offset_aero_absolute)
        .def_readonly("fraction", &seahowl::aero::BladeElementAero::fraction)
        .def_readonly("length", &seahowl::aero::BladeElementAero::length)
        .def_readonly("offset_aero", &seahowl::aero::BladeElementAero::offset_aero);
    py::class_<seahowl::aero::BladeAero, std::shared_ptr<seahowl::aero::BladeAero>, seahowl::fluid::ComponentFluid>(
        m_aero, "BladeAero")
        .def(py::init<>())
        .def("get_total_load", &seahowl::aero::BladeAero::get_total_load)
        .def("get_average_wind_velocity", &seahowl::aero::BladeAero::get_average_wind_velocity)
        .def("compute_distances_from_tip", &seahowl::aero::BladeAero::compute_distances_from_tip)
        .def("compute_distances_from_hub", &seahowl::aero::BladeAero::compute_distances_from_hub)
        .def("compute_radii", &seahowl::aero::BladeAero::compute_radii)
        .def_readwrite("discretization_fractions", &seahowl::aero::BladeAero::discretization_fractions)
        .def_readwrite("reference_points", &seahowl::aero::BladeAero::reference_points)
        .def_readwrite("discretized_points", &seahowl::aero::BladeAero::discretized_points)
        .def_readwrite("nodes", &seahowl::aero::BladeAero::nodes)
        .def_readonly("elements", &seahowl::aero::BladeAero::elements)
        .def_readwrite("azimuth0", &seahowl::aero::BladeAero::azimuth0)
        .def_readwrite("pitch", &seahowl::aero::BladeAero::pitch)
        .def_property_readonly(
            "body_root", [](seahowl::aero::BladeAero& blade) { return blade.body_root.get(); },
            py::return_value_policy::reference_internal);

    // aero/rotor_aero.h
    py::class_<seahowl::aero::RotorAero, std::shared_ptr<seahowl::aero::RotorAero>, seahowl::fluid::ComponentFluid>(
        m_aero, "RotorAero")
        .def("build", &seahowl::aero::RotorAero::build)
        .def("initialize", &seahowl::aero::RotorAero::initialize)
        .def("compute_disk_averaged_wind_velocity", &seahowl::aero::RotorAero::compute_disk_averaged_wind_velocity)
        .def_readwrite("blades", &seahowl::aero::RotorAero::blades)
        .def_readonly("body_hub", &seahowl::aero::RotorAero::body_hub)
        .def_readwrite("hub_radius", &seahowl::aero::RotorAero::hub_radius)
        .def_readwrite("hub_torque_aero", &seahowl::aero::RotorAero::hub_torque_aero)
        .def_readwrite("hub_thrust_aero", &seahowl::aero::RotorAero::hub_thrust_aero)
        .def_readwrite("radius", &seahowl::aero::RotorAero::radius)
        .def_readwrite("azimuth", &seahowl::aero::RotorAero::azimuth)
        .def_readwrite("pitch_collective", &seahowl::aero::RotorAero::pitch_collective)
        .def_readonly("disk_averaged_wind_velocity", &seahowl::aero::RotorAero::disk_averaged_wind_velocity);

    py::class_<seahowl::aero::RotorAeroBET, std::shared_ptr<seahowl::aero::RotorAeroBET>, seahowl::aero::RotorAero>(
        m_aero, "RotorAeroBET");

    py::class_<seahowl::aero::RotorAeroBEMT, std::shared_ptr<seahowl::aero::RotorAeroBEMT>,
               seahowl::aero::RotorAeroBET>(m_aero, "RotorAeroBEMT")
        .def(py::init<seahowl::aero::TowerAero&>())
        .def_readwrite("has_tip_loss", &seahowl::aero::RotorAeroBEMT::has_tip_loss)
        .def_readwrite("has_hub_loss", &seahowl::aero::RotorAeroBEMT::has_hub_loss)
        .def_readwrite("has_tower_shadow", &seahowl::aero::RotorAeroBEMT::has_tower_shadow);

    py::class_<seahowl::aero::RotorAeroDisk, std::shared_ptr<seahowl::aero::RotorAeroDisk>, seahowl::aero::RotorAero>(
        m_aero, "RotorAeroDisk")
        .def(py::init<>())
        .def_readwrite("disk_coefficients", &seahowl::aero::RotorAeroDisk::disk_coefficients)
        .def("build", &seahowl::aero::RotorAeroDisk::build)
        .def("initialize", &seahowl::aero::RotorAeroDisk::initialize);

    py::class_<seahowl::aero::RotorNacelleAssemblyAero, std::shared_ptr<seahowl::aero::RotorNacelleAssemblyAero>,
               seahowl::fluid::ComponentFluid>(m_aero, "RotorNacelleAssemblyAero")
        .def(py::init<>())
        .def("initialize", &seahowl::aero::RotorNacelleAssemblyAero::initialize)
        .def_readwrite("rotor", &seahowl::aero::RotorNacelleAssemblyAero::rotor)
        .def_readonly("body_nacelle", &seahowl::aero::RotorNacelleAssemblyAero::body_nacelle);

    // aero/tower_aero.h
    py::class_<seahowl::aero::TowerAero, std::shared_ptr<seahowl::aero::TowerAero>, seahowl::fluid::ComponentFluid>(
        m_aero, "TowerAero")
        .def(py::init<>())
        .def_readwrite("discretization_fractions", &seahowl::aero::TowerAero::discretization_fractions)
        .def_readwrite("reference_points", &seahowl::aero::TowerAero::reference_points)
        .def_readwrite("discretized_points", &seahowl::aero::TowerAero::discretized_points)
        .def_readwrite("nodes", &seahowl::aero::TowerAero::nodes)
        .def_readonly("elements", &seahowl::aero::TowerAero::elements)
        .def_readwrite("has_nodal_distributed_loads", &seahowl::aero::TowerAero::has_nodal_distributed_loads)
        .def_readwrite("use_MacCamyFuchs_correction", &seahowl::aero::TowerAero::use_MacCamyFuchs_correction)
        .def_readwrite("use_Cd_correction", &seahowl::aero::TowerAero::use_Cd_correction);

    // aero/reference_point_aero.h
    py::class_<seahowl::aero::TowerReferencePointAero, std::shared_ptr<seahowl::aero::TowerReferencePointAero>>(
        m_aero, "TowerReferencePointAero")
        .def(py::init<>())
        .def_readwrite("coordinates", &seahowl::aero::TowerReferencePointAero::coordinates)
        .def_readwrite("fraction", &seahowl::aero::TowerReferencePointAero::fraction)
        .def_readwrite("velocity", &seahowl::aero::TowerReferencePointAero::velocity)
        .def_readwrite("rotation", &seahowl::aero::TowerReferencePointAero::rotation)
        .def_readwrite("diameter", &seahowl::aero::TowerReferencePointAero::diameter)
        .def_readwrite("coefficients", &seahowl::aero::TowerReferencePointAero::coefficients);

#ifdef HAVE_AERODYN
    // aero/aerodyn_adapter.h
    py::class_<seahowl::aero::TurbineAeroDyn, std::shared_ptr<seahowl::aero::TurbineAeroDyn>,
               seahowl::fluid::TurbineFluid>(m_aero, "TurbineAeroDyn")
        .def(py::init<const std::string&>())
        .def_readwrite("WrVTK", &seahowl::aero::TurbineAeroDyn::WrVTK)
        .def_readwrite("WrVTK_Type", &seahowl::aero::TurbineAeroDyn::WrVTK_Type)
        .def_readwrite("WrVTK_dt", &seahowl::aero::TurbineAeroDyn::WrVTK_dt);

    py::class_<seahowl::aero::RotorAeroDyn, std::shared_ptr<seahowl::aero::RotorAeroDyn>, seahowl::aero::RotorAeroBEMT>(
        m_aero, "RotorAeroDyn")
        .def(py::init<seahowl::aero::TowerAero&>());
#endif
}
