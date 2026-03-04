cmake_minimum_required(VERSION 3.10)

set_property(GLOBAL PROPERTY USE_FOLDERS ON)

# ==================
# SEAHOWL PROJECT
# =================
project(
    SEAHOWL
    VERSION 1.0.0
    DESCRIPTION "Servo-Elasto-Aero-Hydro Offshore Wind Lab"
    LANGUAGES CXX
)

# guard against in-source builds
if(${CMAKE_SOURCE_DIR} STREQUAL ${CMAKE_BINARY_DIR})
    message(
        FATAL_ERROR
            "In-source builds not allowed. Please make a new directory (called a build directory) and run CMake from there. You may need to remove CMakeCache.txt. "
    )
endif()

list(APPEND CMAKE_PREFIX_PATH "${CMAKE_INSTALL_PREFIX}/../Chrono/lib")

# Retrieve date and time of build
string(TIMESTAMP TODAY "%Y-%m-%d %H:%M:%S")
set (SEAHOWL_BUILD_DATE ${TODAY})

# Retrieve git commit hash (if any)
execute_process(
    COMMAND git describe --always --dirty
    WORKING_DIRECTORY ${CMAKE_SOURCE_DIR}
    OUTPUT_VARIABLE IS_GIT_REPOSITORY
    OUTPUT_STRIP_TRAILING_WHITESPACE
    ERROR_QUIET
)
if(IS_GIT_REPOSITORY)
    execute_process(
        COMMAND git describe --always --dirty
        WORKING_DIRECTORY ${CMAKE_SOURCE_DIR}
        OUTPUT_VARIABLE SEAHOWL_GIT_HASH
        OUTPUT_STRIP_TRAILING_WHITESPACE
    )
else()
    set(SEAHOWL_GIT_HASH "N/A")
endif()

# =============
# OPTIONS
# =============

option(SEAHOWL_ENABLE_IRRLICHT "Enable Irrlicht 3D visualization library" OFF)
option(SEAHOWL_ENABLE_TESTS "Enable tests" OFF)
option(SEAHOWL_ENABLE_DOC "Generate html documentation" OFF)
option(SEAHOWL_ENABLE_BUILD "Build library and drivers" ON)
option(SEAHOWL_ENABLE_PYTHON "Enable python binding" OFF)
option(SEAHOWL_ENABLE_VIRTUALENV "Enable Python virtual env" OFF)
option(SEAHOWL_ENABLE_EXAMPLES "Enable examples" ON)
option(SEAHOWL_ENABLE_VTK "Enable VTK Library for output" OFF)
option(SEAHOWL_ENABLE_AERODYN "Enable AeroDyn module" OFF)
option(SEAHOWL_ENABLE_INFLOWWIND "Enable InflowWind module" OFF)
option(SEAHOWL_ENABLE_HYDRODYN "Enable HydroDyn module" OFF)
option(SEAHOWL_ENABLE_SEASTATE "Enable SeaState module" OFF)
option(SEAHOWL_ENABLE_HYDROCHRONO "Enable HydroChrono library" OFF)


# =======================
# External libraries
# =======================

# Eigen library
find_package(Eigen3 3.3.0 REQUIRED NO_MODULE)

# Irrlicht Visualization library
if(SEAHOWL_ENABLE_IRRLICHT)
    set(IRRLICHT_COMP Irrlicht)
    add_definitions("-DHAVE_IRRLICHT=1")
endif(SEAHOWL_ENABLE_IRRLICHT)

# Chrono multibody dynamics library
find_package(
    Chrono
    COMPONENTS ${IRRLICHT_COMP}
    OPTIONAL_COMPONENTS PardisoMKL CONFIG
    REQUIRED
)

# JSON Parser library
find_package(nlohmann_json REQUIRED)

# Logging library
find_package(spdlog REQUIRED)

# AeroDyn module
if(SEAHOWL_ENABLE_AERODYN)
    if(WIN32)
        find_library(AERODYN_LIBRARY libaerodyn_inflow_c_binding)
    else()
        find_library(AERODYN_LIBRARY aerodyn_inflow_c_binding)
    endif()
    add_compile_definitions(SEAHOWL_HAVE_AERODYN=1)
    add_definitions("-DHAVE_AERODYN=1")
endif(SEAHOWL_ENABLE_AERODYN)

# InflowWind module
if(SEAHOWL_ENABLE_INFLOWWIND)
    if(WIN32)
        find_library(INFLOWWIND_LIBRARY libifw_c_binding)
    else()
        find_library(INFLOWWIND_LIBRARY ifw_c_binding)
    endif()
    add_compile_definitions(SEAHOWL_HAVE_INFLOWWIND=1)
    add_definitions("-DHAVE_INFLOWWIND=1")
endif(SEAHOWL_ENABLE_INFLOWWIND)

# HydroDyn module
if(SEAHOWL_ENABLE_HYDRODYN)
    if(WIN32)
        find_library(HYDRODYN_LIBRARY libhydrodyn_c_binding)
    else()
        find_library(HYDRODYN_LIBRARY hydrodyn_c_binding)
    endif()
    add_compile_definitions(SEAHOWL_HAVE_HYDRODYN=1)
    add_definitions("-DHAVE_HYDRODYN=1")
endif(SEAHOWL_ENABLE_HYDRODYN)

# SeaState module
if(SEAHOWL_ENABLE_SEASTATE)
    if(WIN32)
	find_library(SEASTATE_LIBRARY libseastate_c_binding)
    else()
	find_library(SEASTATE_LIBRARY seastate_c_binding)
    endif()
    add_compile_definitions(SEAHOWL_HAVE_SEASTATE=1)
    add_definitions("-DHAVE_SEASTATE=1")
endif(SEAHOWL_ENABLE_SEASTATE)

if(NOT CMAKE_SIZEOF_VOID_P EQUAL 8)
	message(FATAL_ERROR "ONLY 64 BITS SUPPORTED")
endif()

if(SEAHOWL_ENABLE_VTK)
    find_package(VTK 9 COMPONENTS CommonDataModel IOXML FiltersGeneral FiltersModeling FiltersSources FiltersExtraction)
endif(SEAHOWL_ENABLE_VTK)

if(VTK_FOUND)
    add_definitions("-DHAVE_VTK=1")
endif()

# HydroChrono potential flow library
if(SEAHOWL_ENABLE_HYDROCHRONO)
    find_package(HydroChrono REQUIRED)
    add_definitions("-DHAVE_HYDROCHRONO=1")
endif(SEAHOWL_ENABLE_HYDROCHRONO)

# python
if(SEAHOWL_ENABLE_PYTHON)
    find_package(Python 3.6 COMPONENTS Interpreter Development REQUIRED)
    find_package(pybind11 CONFIG REQUIRED)
    add_definitions("-DHAVE_PYTHON=1")
endif(SEAHOWL_ENABLE_PYTHON)

IF(SEAHOWL_ENABLE_VIRTUALENV)
     add_definitions("-DHAVE_ENV_PYTHON=1")
endif()

# --------------------------------------------------------------
# Tweaks to disable some warnings with MSVC
# --------------------------------------------------------------
if(MSVC)
    add_definitions("-D_CRT_SECURE_NO_DEPRECATE") # avoids deprecation warnings
    add_definitions("-D_SCL_SECURE_NO_DEPRECATE") # avoids deprecation warnings
    add_definitions("-DNOMINMAX") # do not use MSVC's min/max macros
    add_compile_options(/wd4275) # disable warnings triggered by Irrlicht
    add_compile_options(/wd4251) # class needs to have dll-interface

    # TO BE REMOVED: SHOULD BE ALREADY SET BY CHRONO LIBRARY
    if(${MSVC_VERSION} GREATER_EQUAL 1915) # Fix for VS 2017 15.8 and newer to handle alignment specification with Eigen
        add_definitions("-D_ENABLE_EXTENDED_ALIGNED_STORAGE")
    endif()
else()
    set(EXTRA_COMPILE_FLAGS "")
endif()

# ==============
# DATADIR
# ==============
# Define a data directory to find test input files
#
# If the directory is not set as environment variable set a default value in CMake variable. This default value can be
# modified in the CMakeCache.txt file in build directory.
#
if(DEFINED ENV{SEAHOWL_DATADIR})
    set(SEAHOWL_DATADIR $ENV{SEAHOWL_DATADIR})
else()
    set(SEAHOWL_DATADIR "${PROJECT_SOURCE_DIR}/data")
endif()

message(STATUS "SEAHOWL_DATADIR set to: ${SEAHOWL_DATADIR}")
add_definitions(-DSEAHOWL_DATADIR="${SEAHOWL_DATADIR}")

# ================
# CORE LIBRARY
# ================
add_library(seahowl_core)

# commons
set(SEAHOWL_COMMONS_SOURCES
    src/commons/entities.cpp
    src/commons/utils.cpp
)
set(SEAHOWL_COMMONS_HEADERS
    include/seahowl/seahowl.h
    include/seahowl/commons.h
    include/seahowl/commons/numerics.h
    include/seahowl/commons/entities.h
    include/seahowl/commons/utils.h
)
source_group(commons FILES ${SEAHOWL_COMMONS_SOURCES} ${SEAHOWL_COMMONS_HEADERS})

# core
set(SEAHOWL_CORE_SOURCES
    src/core/reference_point.cpp
    src/core/component.cpp
    src/core/component_elasto_fluid.cpp
    src/core/rotor.cpp
    src/core/blade.cpp
    src/core/tower.cpp
    src/core/mooring.cpp
    src/core/floater.cpp
    src/core/turbine.cpp
    src/core/system.cpp
    src/core/simulation.cpp
)
set(SEAHOWL_CORE_HEADERS
    include/seahowl/core.h
    include/seahowl/core/reference_point.h
    include/seahowl/core/component.h
    include/seahowl/core/component_elasto_fluid.h
    include/seahowl/core/blade.h
    include/seahowl/core/rotor.h
    include/seahowl/core/tower.h
    include/seahowl/core/mooring.h
    include/seahowl/core/foundation.h
    include/seahowl/core/floater.h
    include/seahowl/core/monopile.h
    include/seahowl/core/turbine.h
    include/seahowl/core/system.h
    include/seahowl/core/simulation.h
)
source_group(core FILES ${SEAHOWL_CORE_SOURCES} ${SEAHOWL_CORE_HEADERS})

# elasto
set(SEAHOWL_ELASTO_SOURCES
    src/elasto/entities_elasto.cpp
    src/elasto/component_elasto.cpp
    src/elasto/system_elasto.cpp
    src/elasto/blade_elasto.cpp
    src/elasto/reference_point_elasto.cpp
    src/elasto/rotor_elasto.cpp
    src/elasto/tower_elasto.cpp
    src/elasto/foundation_elasto.cpp
    src/elasto/floater_elasto.cpp
    src/elasto/mooring_elasto.cpp
    src/elasto/monopile_elasto.cpp
    src/elasto/turbine_elasto.cpp
    src/elasto/chrono_adapters.cpp
)
set(SEAHOWL_ELASTO_HEADERS
    include/seahowl/elasto.h
    include/seahowl/elasto/blade_elasto.h
    include/seahowl/elasto/entities_elasto.h
    include/seahowl/elasto/component_elasto.h
    include/seahowl/elasto/system_elasto.h
    include/seahowl/elasto/reference_point_elasto.h
    include/seahowl/elasto/rotor_elasto.h
    include/seahowl/elasto/tower_elasto.h
    include/seahowl/elasto/foundation_elasto.h
    include/seahowl/elasto/floater_elasto.h
    include/seahowl/elasto/mooring_elasto.h
    include/seahowl/elasto/monopile_elasto.h
    include/seahowl/elasto/turbine_elasto.h
    include/seahowl/elasto/chrono_adapters.h
)
source_group(elasto FILES ${SEAHOWL_ELASTO_SOURCES} ${SEAHOWL_ELASTO_HEADERS})

# fluid (shared between aero and hydro)
set(SEAHOWL_FLUID_SOURCES
    src/fluid/system_fluid.cpp
    src/fluid/turbine_fluid.cpp
)
set(SEAHOWL_FLUID_HEADERS
    include/seahowl/fluid.h
    include/seahowl/fluid/component_fluid.h
    include/seahowl/fluid/system_fluid.h
    include/seahowl/fluid/turbine_fluid.h
)
source_group(fluid FILES ${SEAHOWL_FLUID_SOURCES} ${SEAHOWL_FLUID_HEADERS})

# aero
set(SEAHOWL_AERO_SOURCES
    src/fluid/aero/airfoil.cpp
    src/fluid/aero/bemt.cpp
    src/fluid/aero/reference_point_aero.cpp
    src/fluid/aero/blade_aero.cpp
    src/fluid/aero/rotor_aero.cpp
    src/fluid/aero/tower_aero.cpp
)
set(SEAHOWL_AERO_HEADERS
    include/seahowl/aero.h
    include/seahowl/fluid/aero/airfoil.h
    include/seahowl/fluid/aero/bemt.h
    include/seahowl/fluid/aero/blade_aero.h
    include/seahowl/fluid/aero/reference_point_aero.h
    include/seahowl/fluid/aero/rotor_aero.h
    include/seahowl/fluid/aero/tower_aero.h
)
if (SEAHOWL_ENABLE_AERODYN)
    list(APPEND SEAHOWL_AERO_SOURCES src/fluid/aero/aerodyn_adapter.cpp)
    list(APPEND SEAHOWL_AERO_HEADERS include/seahowl/fluid/aero/aerodyn_adapter.h)
endif(SEAHOWL_ENABLE_AERODYN)
source_group(aero FILES
    ${SEAHOWL_AERO_SOURCES}
    ${SEAHOWL_AERO_HEADERS})

# servo
set(SEAHOWL_SERVO_SOURCES
    src/servo/controller.cpp
    src/servo/controller_discon.cpp
)
set(SEAHOWL_SERVO_HEADERS
    include/seahowl/servo.h
    include/seahowl/servo/controller.h
    include/seahowl/servo/controller_discon.h
)
source_group(servo FILES ${SEAHOWL_SERVO_SOURCES} ${SEAHOWL_SERVO_HEADERS})

# hydro
set(SEAHOWL_HYDRO_SOURCES
    src/fluid/hydro/morison.cpp
    src/fluid/hydro/mooring_hydro.cpp
    src/fluid/hydro/floater_hydro.cpp
)
set(SEAHOWL_HYDRO_HEADERS
    include/seahowl/hydro.h
    include/seahowl/fluid/hydro/morison.h
    include/seahowl/fluid/hydro/mooring_hydro.h
    include/seahowl/fluid/hydro/foundation_fluid.h
    include/seahowl/fluid/hydro/floater_hydro.h
    include/seahowl/fluid/hydro/monopile_hydro.h
)
if(SEAHOWL_ENABLE_HYDROCHRONO)
    list(APPEND SEAHOWL_HYDRO_SOURCES src/fluid/hydro/hydrochrono_adapter.cpp)
    list(APPEND SEAHOWL_HYDRO_HEADERS include/seahowl/fluid/hydro/hydrochrono_adapter.h)
endif(SEAHOWL_ENABLE_HYDROCHRONO)
if (SEAHOWL_ENABLE_HYDRODYN)
    list(APPEND SEAHOWL_AERO_SOURCES src/fluid/hydro/hydrodyn_adapter.cpp)
    list(APPEND SEAHOWL_AERO_HEADERS include/seahowl/fluid/hydro/hydrodyn_adapter.h)
endif(SEAHOWL_ENABLE_HYDRODYN)
source_group(hydro FILES ${SEAHOWL_HYDRO_SOURCES} ${SEAHOWL_HYDRO_HEADERS})

# env
set(SEAHOWL_ENV_SOURCES
    src/env/env_model.cpp
    src/env/fluid_list_model.cpp
    src/env/soil_list_model.cpp
    src/env/fluid_models.cpp
    src/env/wind_models.cpp
    src/env/wave_models.cpp
    src/env/soil_models.cpp
)
set(SEAHOWL_ENV_HEADERS
    include/seahowl/env.h
    include/seahowl/env/model.h
    include/seahowl/env/list_model.h
    include/seahowl/env/fluid_list_model.h
    include/seahowl/env/soil_list_model.h
    include/seahowl/env/env_model.h
    include/seahowl/env/fluid_models.h
    include/seahowl/env/wind_models.h
    include/seahowl/env/wave_models.h
    include/seahowl/env/soil_models.h
)
if (SEAHOWL_ENABLE_INFLOWWIND)
    list(APPEND SEAHOWL_ENV_SOURCES src/env/inflowwind_adapter.cpp)
    list(APPEND SEAHOWL_ENV_HEADERS include/seahowl/env/inflowwind_adapter.h)
endif(SEAHOWL_ENABLE_INFLOWWIND)
if (SEAHOWL_ENABLE_SEASTATE)
    list(APPEND SEAHOWL_ENV_SOURCES src/env/seastate_adapter.cpp)
    list(APPEND SEAHOWL_ENV_HEADERS include/seahowl/env/seastate_adapter.h)
endif(SEAHOWL_ENABLE_SEASTATE)
source_group(env FILES ${SEAHOWL_ENV_SOURCES} ${SEAHOWL_ENV_HEADERS})

# io
set(SEAHOWL_IO_SOURCES
    src/io/read_input.cpp
    src/io/write_csv.cpp
    src/io/read_rotor_perf.cpp
    src/io/utils_io.cpp
	src/io/viz_insitu.cpp
	src/io/output_manager.cpp
    src/io/config_manager.cpp
    src/io/command_parser.cpp
    src/io/input_reader_json.cpp
    src/io/input_handler.cpp
)
set(SEAHOWL_IO_HEADERS
    include/seahowl/io.h
    include/seahowl/io/read_input.h
    include/seahowl/io/write_csv.h
    include/seahowl/io/read_rotor_perf.h
    include/seahowl/io/utils_io.h
	include/seahowl/io/viz_insitu.h
	include/seahowl/io/output_manager.h
    include/seahowl/io/config_manager.h
    include/seahowl/io/command_parser.h
    include/seahowl/io/input_structures.h
    include/seahowl/io/input_handler.h
    include/seahowl/io/input_reader.h
    include/seahowl/io/input_reader_json.h
)

if(SEAHOWL_ENABLE_VTK)
    list(APPEND SEAHOWL_IO_SOURCES src/io/write_vtk.cpp)
    list(APPEND SEAHOWL_IO_HEADERS include/seahowl/io/write_vtk.h)
endif(SEAHOWL_ENABLE_VTK)
if(SEAHOWL_ENABLE_IRRLICHT)
    list(APPEND SEAHOWL_IO_SOURCES src/io/viz_insitu_irrlicht.cpp)
    list(APPEND SEAHOWL_IO_HEADERS include/seahowl/io/viz_insitu_irrlicht.h)
endif(SEAHOWL_ENABLE_IRRLICHT)
source_group(io FILES ${SEAHOWL_IO_SOURCES} ${SEAHOWL_IO_HEADERS})

# seahowl lib
set(SEAHOWL_FILES
    ${SEAHOWL_COMMONS_SOURCES}
    ${SEAHOWL_COMMONS_HEADERS}
    ${SEAHOWL_CORE_SOURCES}
    ${SEAHOWL_CORE_HEADERS}
    ${SEAHOWL_ELASTO_SOURCES}
    ${SEAHOWL_ELASTO_HEADERS}
    ${SEAHOWL_FLUID_SOURCES}
    ${SEAHOWL_FLUID_HEADERS}
    ${SEAHOWL_AERO_SOURCES}
    ${SEAHOWL_AERO_HEADERS}
    ${SEAHOWL_SERVO_SOURCES}
    ${SEAHOWL_SERVO_HEADERS}
    ${SEAHOWL_HYDRO_SOURCES}
    ${SEAHOWL_HYDRO_HEADERS}
    ${SEAHOWL_ENV_SOURCES}
    ${SEAHOWL_ENV_HEADERS}
    ${SEAHOWL_IO_SOURCES}
    ${SEAHOWL_IO_HEADERS}
)

target_sources(
    seahowl_core
    PRIVATE ${SEAHOWL_FILES}
)

set_target_properties(
    seahowl_core
    PROPERTIES
    POSITION_INDEPENDENT_CODE ON
	)

target_include_directories(
    seahowl_core
    PUBLIC $<BUILD_INTERFACE:${PROJECT_SOURCE_DIR}/include> $<INSTALL_INTERFACE:include>
    PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/ ${CMAKE_CURRENT_SOURCE_DIR}/include ${CHRONO_INCLUDE_DIRS} ${PROJECT_BINARY_DIR}
)

target_compile_features(
    seahowl_core
    PRIVATE cxx_std_17
)

target_compile_definitions(
    seahowl_core
    PRIVATE
    CHRONO_DATA_DIR="${CHRONO_DATA_DIR}"
    SEAHOWL_HAVE_HYDROCHRONO=$<BOOL:${SEAHOWL_ENABLE_HYDROCHRONO}>
    SEAHOWL_HAVE_VTK=$<BOOL:${SEAHOWL_ENABLE_VTK}>
    SEAHOWL_HAVE_AERODYN=$<BOOL:${SEAHOWL_ENABLE_AERODYN}>
    SEAHOWL_HAVE_INFLOWWIND=$<BOOL:${SEAHOWL_ENABLE_INFLOWWIND}>
    SEAHOWL_HAVE_HYDRODYN=$<BOOL:${SEAHOWL_ENABLE_HYDRODYN}>
    SEAHOWL_HAVE_SEASTATE=$<BOOL:${SEAHOWL_ENABLE_SEASTATE}>
    SEAHOWL_HAVE_IRRLICHT=$<BOOL:${SEAHOWL_ENABLE_IRRLICHT}>
    SEAHOWL_CMAKE_BUILD_TYPE="${CMAKE_BUILD_TYPE}"
    SEAHOWL_NAME="${PROJECT_NAME}"
    SEAHOWL_DESCRIPTION="${PROJECT_DESCRIPTION}"
    SEAHOWL_VERSION="${SEAHOWL_VERSION}"
    SEAHOWL_BUILD_DATE="${SEAHOWL_BUILD_DATE}"
    SEAHOWL_GIT_HASH="${SEAHOWL_GIT_HASH}"
    SEAHOWL_COMPILER="${CMAKE_CXX_COMPILER_ID} ${CMAKE_CXX_COMPILER_VERSION}"
    SEAHOWL_BUILD_ARCHITECTURE="${CMAKE_SYSTEM_PROCESSOR} ${CMAKE_SYSTEM_NAME}"
)

target_compile_options(
    seahowl_core
    PRIVATE ${CHRONO_CXX_FLAGS}
)

target_link_options(
    seahowl_core
    PRIVATE ${CHRONO_LINKER_FLAGS}
)

target_link_libraries(seahowl_core
    PUBLIC Eigen3::Eigen
    PRIVATE nlohmann_json::nlohmann_json ${CHRONO_LIBRARIES} spdlog::spdlog
)

if (SEAHOWL_ENABLE_HYDROCHRONO)
    target_link_libraries(seahowl_core PRIVATE HydroChrono::HydroChrono)
endif (SEAHOWL_ENABLE_HYDROCHRONO)

if(SEAHOWL_ENABLE_VTK)
    target_link_libraries(seahowl_core PRIVATE VTK::CommonDataModel VTK::IOXML)
endif(SEAHOWL_ENABLE_VTK)

if (SEAHOWL_ENABLE_AERODYN)
    target_link_libraries(seahowl_core PRIVATE ${AERODYN_LIBRARY})
endif (SEAHOWL_ENABLE_AERODYN)

if (SEAHOWL_ENABLE_INFLOWWIND)
    target_link_libraries(seahowl_core PRIVATE ${INFLOWWIND_LIBRARY})
endif (SEAHOWL_ENABLE_INFLOWWIND)

if (SEAHOWL_ENABLE_HYDRODYN)
    target_link_libraries(seahowl_core PRIVATE ${HYDRODYN_LIBRARY})
endif (SEAHOWL_ENABLE_HYDRODYN)

if (SEAHOWL_ENABLE_SEASTATE)
    target_link_libraries(seahowl_core PRIVATE ${SEASTATE_LIBRARY})
endif (SEAHOWL_ENABLE_SEASTATE)

if(SEAHOWL_ENABLE_IRRLICHT)
    # the following is needed when building with vcpkg (link stage)
    find_package(PNG CONFIG REQUIRED)
    find_package(JPEG REQUIRED)
    find_package(BZip2 REQUIRED)
    find_package(Irrlicht CONFIG REQUIRED)
    target_link_libraries(seahowl_core PRIVATE PNG::PNG JPEG::JPEG BZip2::BZip2 Irrlicht)
endif(SEAHOWL_ENABLE_IRRLICHT)


# =============
# DRIVER
# =============
add_executable(seahowl_driver)

target_sources(
    seahowl_driver PRIVATE src/main.cpp
)

target_include_directories(
    seahowl_driver
    PUBLIC $<BUILD_INTERFACE:${PROJECT_SOURCE_DIR}/include> $<INSTALL_INTERFACE:include>
    PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/ ${CMAKE_CURRENT_SOURCE_DIR}/include ${CHRONO_INCLUDE_DIRS}
)

target_compile_features(seahowl_driver PRIVATE cxx_std_17)

target_compile_options(seahowl_driver PRIVATE ${CHRONO_CXX_FLAGS})

target_link_libraries(seahowl_driver PRIVATE seahowl_core nlohmann_json::nlohmann_json ${CHRONO_LIBRARIES} spdlog::spdlog)

if (SEAHOWL_ENABLE_HYDROCHRONO)
    target_link_libraries(seahowl_driver PRIVATE HydroChrono::HydroChrono)
endif (SEAHOWL_ENABLE_HYDROCHRONO)

# --------------------------------------------------------------
# === 4 (OPTIONAL) ===
#
# Optionally, add a custom command for copying all Chrono and dependency DLLs to the appropriate binary output folder.
# This function has effect only on Windows.
#
# Note that you must first set EXECUTABLE_OUTPUT_PATH (this can simply be ${CMAKE_BINARY_DIR}, like in this example)
# --------------------------------------------------------------

set(EXECUTABLE_OUTPUT_PATH ${CMAKE_BINARY_DIR})
add_dll_copy_command("${CHRONO_DLLS}")

# ============
# Examples
# ===========
if(SEAHOWL_ENABLE_EXAMPLES)
    add_subdirectory(examples)
endif(SEAHOWL_ENABLE_EXAMPLES)

# ===========
# TESTS
# ===========
if(SEAHOWL_ENABLE_TESTS)

    include(CTest)
    enable_testing()

    add_subdirectory(tests)

endif(SEAHOWL_ENABLE_TESTS)


# ====================
# PYTHON
# ====================
if (SEAHOWL_ENABLE_PYTHON)
    set(PYSEAHOWL_DIR ${CMAKE_CURRENT_SOURCE_DIR}/src/bindings/python)
    pybind11_add_module(
        pyseahowl
        ${PYSEAHOWL_DIR}/pyseahowl.cpp
        ${PYSEAHOWL_DIR}/pyseahowl_env.cpp
        ${PYSEAHOWL_DIR}/pyseahowl_elasto.cpp
        ${PYSEAHOWL_DIR}/pyseahowl_fluid.cpp
        ${PYSEAHOWL_DIR}/pyseahowl_aero.cpp
        ${PYSEAHOWL_DIR}/pyseahowl_hydro.cpp
        ${PYSEAHOWL_DIR}/pyseahowl_servo.cpp
        ${PYSEAHOWL_DIR}/pyseahowl_core.cpp
        ${PYSEAHOWL_DIR}/pyseahowl_io.cpp
    )
	target_compile_features(pyseahowl PRIVATE cxx_std_17)
    target_link_libraries(pyseahowl PRIVATE seahowl_core pybind11::embed)
    target_compile_options(pyseahowl PRIVATE ${CHRONO_CXX_FLAGS})
    set_target_properties(pyseahowl PROPERTIES OUTPUT_NAME seahowl)


    IF(SEAHOWL_ENABLE_VIRTUALENV)
        execute_process(
                COMMAND ${Python_EXECUTABLE} -m ensurepip
                RESULT_VARIABLE result
        )

        if(result)
            message(FATAL_ERROR "Failed to install pip")
        endif()

        execute_process(
                COMMAND ${Python_EXECUTABLE} -m pip install virtualenv
                RESULT_VARIABLE result
        )

        if(result)
            message(FATAL_ERROR "Failed to install virtualenv")
        endif()

        if(NOT EXISTS ${CMAKE_BINARY_DIR}/.venv/pyvenv.cfg)
            execute_process(
                    COMMAND ${Python_EXECUTABLE} -m virtualenv ${CMAKE_BINARY_DIR}/.venv
                    RESULT_VARIABLE result
            )
        endif()

        if(result)
            message(FATAL_ERROR "Failed to create virtual environment")
        endif()

        set(CMD_PIP "pip install mkdocs mkdocs-material mkdocs-include-markdown-plugin pybind11-stubgen \'mkdocstrings[python]\' matplotlib myst-parser pre-commit numpy pytest scilens scilens-compare")
        set(CMD_PRECOMMIT "pre-commit install")

        if(${CMAKE_SYSTEM_NAME} MATCHES "Linux")
            set(ACTIVATE_PY "source ${CMAKE_BINARY_DIR}/.venv/bin/activate")
            set(LAUNCH_PIP bash -c)
        else()
            set(ACTIVATE_PY "${CMAKE_BINARY_DIR}/.venv/Scripts/activate")
            set(LAUNCH_PIP cmd.exe /c)
        endif()

        execute_process(
                COMMAND ${LAUNCH_PIP} "${ACTIVATE_PY} && ${CMD_PIP} && ${CMD_PRECOMMIT}"
            )
    endif(SEAHOWL_ENABLE_VIRTUALENV)

endif (SEAHOWL_ENABLE_PYTHON)


# ====================
# DOCUMENTATION
# ====================
if(SEAHOWL_ENABLE_DOC)
    add_subdirectory(doc)
endif(SEAHOWL_ENABLE_DOC)

# ===========================
# Copy the DISCON library
# ===========================
if (VCPKG_APPLOCAL_DEPS)
    if(${CMAKE_SYSTEM_NAME} MATCHES "Linux")
        set(LIB_FILE_NAME "libdiscon.so")
    elseif(APPLE)
        set(LIB_FILE_NAME "libdiscon.dylib")
    else()
        set(LIB_FILE_NAME "libdiscon.dll")
    endif()

    set(LIBRARY_PATH "${VCPKG_INSTALLED_DIR}/${VCPKG_TARGET_TRIPLET}/lib/${LIB_FILE_NAME}")
    set(DESTINATION_PATH_IEA15 "${SEAHOWL_DATADIR}/IEA15MW/base/controller")
    set(DESTINATION_PATH_IEA34 "${SEAHOWL_DATADIR}/IEA34MW/controller")
    set(DESTINATION_PATH_IEA10 "${SEAHOWL_DATADIR}/IEA10MW/controller")

    if(EXISTS ${LIBRARY_PATH})
        file(COPY ${LIBRARY_PATH} DESTINATION ${DESTINATION_PATH_IEA15})
        file(RENAME "${DESTINATION_PATH_IEA15}/${LIB_FILE_NAME}" "${DESTINATION_PATH_IEA15}/libdiscon.so")
        file(COPY ${LIBRARY_PATH} DESTINATION ${DESTINATION_PATH_IEA34})
        file(RENAME "${DESTINATION_PATH_IEA34}/${LIB_FILE_NAME}" "${DESTINATION_PATH_IEA34}/libdiscon.so")
        file(COPY ${LIBRARY_PATH} DESTINATION ${DESTINATION_PATH_IEA10})
        file(RENAME "${DESTINATION_PATH_IEA10}/${LIB_FILE_NAME}" "${DESTINATION_PATH_IEA10}/libdiscon.so")
    else()
        message(WARNING "Library ${LIBRARY_PATH} does not exist.")
    endif()
endif()

# ===========================
# INSTALL
# ===========================
install(TARGETS seahowl_core
        ARCHIVE DESTINATION lib
        LIBRARY DESTINATION lib
        RUNTIME DESTINATION bin)

install(TARGETS seahowl_driver
        RUNTIME DESTINATION bin)

if(NOT WIN32)
    set_target_properties(seahowl_driver PROPERTIES
                          INSTALL_RPATH "$ORIGIN/../lib"
                          BUILD_RPATH "$ORIGIN/../lib")
endif()

if(SEAHOWL_ENABLE_PYTHON)
    if(NOT WIN32)
        install(TARGETS pyseahowl
                LIBRARY DESTINATION lib)

        set_target_properties(pyseahowl PROPERTIES
                            INSTALL_RPATH "$ORIGIN"
                            BUILD_RPATH "$ORIGIN")
    else()
        install(TARGETS pyseahowl
                LIBRARY DESTINATION bin)
    endif()

    if (VCPKG_APPLOCAL_DEPS)
        install(DIRECTORY ${VCPKG_INSTALLED_DIR}/${VCPKG_TARGET_TRIPLET}/tools/python3
                            DESTINATION tools
                            USE_SOURCE_PERMISSIONS)

        if(${CMAKE_SYSTEM_NAME} MATCHES "Linux")
            set(FILE_INITPY ${CMAKE_BINARY_DIR}/init_pyseahowl.sh)
            file(WRITE ${FILE_INITPY}
                "#!/bin/bash\n"
                "SCRIPT_DIR=$(cd \"$(dirname \"\${BASH_SOURCE}\")\" && pwd)\n"
                "SCRIPT_DIR=$(dirname \"$SCRIPT_DIR\")\n"
                "sed -i 's|^prefix=.*|prefix=\"\$prefix_real\"|' \"$SCRIPT_DIR/tools/python3/python3.12-config\"\n"
                "sed -i 's|^exec_prefix=.*|exec_prefix=\"\$prefix_real\"|' \"$SCRIPT_DIR/tools/python3/python3.12-config\"\n"
                "\"$SCRIPT_DIR/tools/python3/python3\" -m virtualenv .venv \n"
                "CALL_DIR=$(pwd)\n"
                "source \"$CALL_DIR/.venv/bin/activate\"\n"
                "export PYTHONPATH=\"$SCRIPT_DIR/lib\":$PYTHONPATH\n")
        else()
            set(FILE_INITPY ${CMAKE_BINARY_DIR}/init_pyseahowl.bat)
            file(WRITE ${FILE_INITPY}
                "set SCRIPT_DIR=%~dp0\n"
                "for %%i in (\"%SCRIPT_DIR%..\") do set SCRIPT_DIR=%%~fi\n"
                "\"%SCRIPT_DIR%/tools/python3/python.exe\" -m virtualenv \"%SCRIPT_DIR%/.venv\"\n"
                "call \"%SCRIPT_DIR%/.venv/Scripts/activate\"\n"
                "set PYTHONPATH=%SCRIPT_DIR%/bin;%PYTHONPATH%\n"
                "cmd /K\n")
            set(FILE_INITPYPS ${CMAKE_BINARY_DIR}/init_pyseahowl.ps1)
                file(WRITE ${FILE_INITPYPS}
                    "$SCRIPT_DIR = Split-Path -Parent $MyInvocation.MyCommand.Definition\n"
                    "$SCRIPT_DIR = Split-Path -Parent $SCRIPT_DIR\n"
                    "& \"$SCRIPT_DIR\\tools\\python3\\python.exe\" -m virtualenv \"$SCRIPT_DIR\\.venv\"\n"
                    "& \"$SCRIPT_DIR\\.venv\\Scripts\\Activate.ps1\"\n"
                    "$env:PYTHONPATH = \"$SCRIPT_DIR\\bin;$env:PYTHONPATH\"\n")

            file(CHMOD ${FILE_INITPYPS} FILE_PERMISSIONS OWNER_READ OWNER_WRITE OWNER_EXECUTE GROUP_READ GROUP_EXECUTE WORLD_READ WORLD_EXECUTE)

            install(FILES ${FILE_INITPYPS}
                DESTINATION tools
                PERMISSIONS OWNER_READ OWNER_WRITE OWNER_EXECUTE)
        endif()

        file(CHMOD ${FILE_INITPY} FILE_PERMISSIONS OWNER_READ OWNER_WRITE OWNER_EXECUTE GROUP_READ GROUP_EXECUTE WORLD_READ WORLD_EXECUTE)

        install(FILES ${FILE_INITPY}
            DESTINATION tools
            PERMISSIONS OWNER_READ OWNER_WRITE OWNER_EXECUTE)
    endif(VCPKG_APPLOCAL_DEPS)

endif(SEAHOWL_ENABLE_PYTHON)

if(${CMAKE_SYSTEM_NAME} MATCHES "Linux")
    INSTALL(CODE "
        include(BundleUtilities)
        fixup_bundle(\"${CMAKE_BINARY_DIR}/seahowl_driver\" \"\" \"${VCPKG_INSTALLED_DIR}/${VCPKG_TARGET_TRIPLET}/lib\")
    " DESTINATION bin COMPONENT Runtime)
    file(GLOB SO_FILES "${CMAKE_BINARY_DIR}/*.so")
    INSTALL(FILES ${SO_FILES} DESTINATION lib)
else()

    if (${CMAKE_GENERATOR} MATCHES "Ninja")
        set(FILTER_FILE ${CMAKE_BINARY_DIR}/*.dll)
        file(GLOB LIST_DLL ${FILTER_FILE})
    else()
        set(FILTER_FILE1 ${CMAKE_BINARY_DIR}/${CMAKE_BUILD_TYPE})
        set(FILTER_FILE ${FILTER_FILE1}/*.dll)
        file(GLOB LIST_DLL ${FILTER_FILE})
    endif()

    install(FILES ${LIST_DLL}
            DESTINATION bin)
endif()

install(DIRECTORY ${SEAHOWL_DATADIR}
        DESTINATION .)

#copy the README.md file to the install directory
install(FILES ${PROJECT_SOURCE_DIR}/doc/source/user-guide/using_package.md
        DESTINATION .)
#rename the using_package.md file to README.md
install(CODE "
        file(RENAME \"${CMAKE_INSTALL_PREFIX}/using_package.md\" \"${CMAKE_INSTALL_PREFIX}/README.md\")
    ")
