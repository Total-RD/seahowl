#include <pybind11/pybind11.h>
#include <pybind11/stl.h>
#include <pybind11/eigen.h>

#include <seahowl/env/wind_models.h>
#ifdef HAVE_INFLOWWIND
    #include <seahowl/env/inflowwind_adapter.h>
#endif

namespace py = pybind11;

void initialize_pyseahowl_env(py::module& m) {
    // submodule
    auto m_aero = m.def_submodule("env", "Env submodule.");

    // env/fluid_models.h
    py::class_<seahowl::env::FluidModel, std::shared_ptr<seahowl::env::FluidModel>>(m_aero, "FluidModel")
        .def("get_fluid_velocity", &seahowl::env::FluidModel::get_fluid_velocity)
        .def("get_fluid_density", &seahowl::env::FluidModel::get_fluid_density);

    // env/wind_models.h
    py::class_<seahowl::env::WindModel, std::shared_ptr<seahowl::env::WindModel>, seahowl::env::FluidModel>(m_aero,
                                                                                                            "WindModel")
        .def_readwrite("air_density", &seahowl::env::WindModel::density);
    py::class_<seahowl::env::ShearedWind, std::shared_ptr<seahowl::env::ShearedWind>, seahowl::env::WindModel>(
        m_aero, "ShearedWind")
        .def_readwrite("shear_coefficient", &seahowl::env::ShearedWind::shear_coefficient)
        .def_readwrite("reference_height", &seahowl::env::ShearedWind::reference_height)
        .def_readwrite("reference_length", &seahowl::env::ShearedWind::reference_length)
        .def_readwrite("direction_gravity", &seahowl::env::ShearedWind::direction_gravity);
    py::class_<seahowl::env::ConstantWind, std::shared_ptr<seahowl::env::ConstantWind>, seahowl::env::ShearedWind>(
        m_aero, "ConstantWind")
        .def(py::init<>())
        .def("set_wind_velocity", &seahowl::env::ConstantWind::set_wind_velocity)
        .def_readwrite("shear_coefficient", &seahowl::env::ConstantWind::shear_coefficient);
    py::class_<seahowl::env::WindRamp, std::shared_ptr<seahowl::env::WindRamp>, seahowl::env::ShearedWind>(m_aero,
                                                                                                           "WindRamp")
        .def(py::init<>())
        .def("set_wind_ramp", &seahowl::env::WindRamp::set_wind_ramp)
        .def_readwrite("time_start", &seahowl::env::WindRamp::time_start)
        .def_readwrite("time_end", &seahowl::env::WindRamp::time_end)
        .def_readwrite("wind_velocity_start", &seahowl::env::WindRamp::wind_velocity_start)
        .def_readwrite("wind_velocity_end", &seahowl::env::WindRamp::wind_velocity_end);

#ifdef HAVE_INFLOWWIND
    // env/infflowwind_adapter.h
    py::class_<seahowl::env::InflowWindAdapter, std::shared_ptr<seahowl::env::InflowWindAdapter>,
               seahowl::env::FluidModel>(m_aero, "InflowWindAdapter")
        .def(py::init<std::string&>());
#endif
}
