#include <pybind11/pybind11.h>
#include <pybind11/stl.h>
#include <pybind11/eigen.h>

#include <seahowl/env/wind_models.h>
#include <seahowl/env/wave_models.h>
#ifdef HAVE_INFLOWWIND
    #include <seahowl/env/inflowwind_adapter.h>
#endif
#ifdef HAVE_HYDROCHRONO
    #include <seahowl/hydro/hydrochrono_adapter.h>
#endif

namespace py = pybind11;

void initialize_pyseahowl_env(py::module& m) {
    // submodule
    auto m_env = m.def_submodule("env", "Env submodule.");

    // env/fluid_models.h
    py::class_<seahowl::env::FluidModel, std::shared_ptr<seahowl::env::FluidModel>>(m_env, "FluidModel")
        .def("get_fluid_velocity", &seahowl::env::FluidModel::get_fluid_velocity)
        .def("get_fluid_density", &seahowl::env::FluidModel::get_fluid_density);

    // env/wind_models.h
    py::class_<seahowl::env::WindModel, std::shared_ptr<seahowl::env::WindModel>, seahowl::env::FluidModel>(m_env,
                                                                                                            "WindModel")
        .def_readwrite("air_density", &seahowl::env::WindModel::density);
    py::class_<seahowl::env::ShearedWind, std::shared_ptr<seahowl::env::ShearedWind>, seahowl::env::WindModel>(
        m_env, "ShearedWind")
        .def_readwrite("shear_coefficient", &seahowl::env::ShearedWind::shear_coefficient)
        .def_readwrite("reference_height", &seahowl::env::ShearedWind::reference_height)
        .def_readwrite("direction_gravity", &seahowl::env::ShearedWind::direction_gravity);
    py::class_<seahowl::env::ConstantWind, std::shared_ptr<seahowl::env::ConstantWind>, seahowl::env::ShearedWind>(
        m_env, "ConstantWind")
        .def(py::init<>())
        .def("set_wind_velocity", &seahowl::env::ConstantWind::set_wind_velocity)
        .def_readwrite("shear_coefficient", &seahowl::env::ConstantWind::shear_coefficient);
    py::class_<seahowl::env::WindRamp, std::shared_ptr<seahowl::env::WindRamp>, seahowl::env::ShearedWind>(m_env,
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
               seahowl::env::FluidModel>(m_env, "InflowWindAdapter")
        .def(py::init<std::string&>());
#endif

    // env/wave_models.h
    py::class_<seahowl::env::WaveModel, std::shared_ptr<seahowl::env::WaveModel>, seahowl::env::FluidModel>(m_env,
                                                                                                            "WaveModel")
        .def_readwrite("density", &seahowl::env::WaveModel::density)
        .def_readwrite("mean_water_level", &seahowl::env::WaveModel::mean_water_level)
        .def_readwrite("surface_normal", &seahowl::env::WaveModel::surface_normal)
        .def_readwrite("water_depth", &seahowl::env::WaveModel::water_depth);
    py::class_<seahowl::env::StillWater, std::shared_ptr<seahowl::env::StillWater>, seahowl::env::WaveModel>(
        m_env, "StillWater")
        .def(py::init<>());
    py::class_<seahowl::env::CurrentConstant, std::shared_ptr<seahowl::env::CurrentConstant>, seahowl::env::WaveModel>(
        m_env, "CurrentConstant")
        .def(py::init<>())
        .def_readwrite("velocity_surface", &seahowl::env::CurrentConstant::velocity_surface)
        .def_readwrite("velocity_seabed", &seahowl::env::CurrentConstant::velocity_seabed)
        .def_readwrite("direction", &seahowl::env::CurrentConstant::direction)
        .def_readwrite("power_factor", &seahowl::env::CurrentConstant::power_factor);
#ifdef HAVE_HYDROCHRONO
    py::class_<seahowl::env::WaveModelHydroChrono, std::shared_ptr<seahowl::env::WaveModelHydroChrono>,
               seahowl::env::WaveModel>(m_env, "WaveModelHydroChrono")
        .def(py::init<>());
#endif
}
