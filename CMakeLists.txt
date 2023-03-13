cmake_minimum_required(VERSION 3.10)

set_property(GLOBAL PROPERTY USE_FOLDERS ON)

# ==================
# SEAHOWL PROJECT
# =================
project(
    SEAHOWL
    VERSION 0.3.0
    DESCRIPTION "SEAHOWL: Servo-Elasto-Aero-Hydro Offsore Wind Lab"
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

# Add the cmake folder so the FindSphinx module is found
set(CMAKE_MODULE_PATH "${PROJECT_SOURCE_DIR}/cmake" ${CMAKE_MODULE_PATH})

# =============
# OPTIONS
# =============

option(SEAHOWL_ENABLE_IRRLICHT "Enable Irrlicht 3D visualization library" OFF)
option(SEAHOWL_ENABLE_TESTS "Enable tests" OFF)
option(SEAHOWL_ENABLE_DOC "Generate html documentation" OFF)
option(SEAHOWL_ENABLE_BUILD "Build library and drivers" ON)
option(SEAHOWL_ENABLE_PYTHON "Enable python binding" OFF)
option(SEAHOWL_ENABLE_EXAMPLES "Enable examples" ON)
option(SEAHOWL_ENABLE_VTK "Enable VTK Library for output" OFF)
option(SEAHOWL_ENABLE_AERODYN "Enable AeroDyn module" OFF)
option(SEAHOWL_ENABLE_INFLOWWIND "Enable InflowWind module" OFF)

# =======================
# External libraries
# =======================

# Eigen library
find_package(Eigen3 3.3.0 REQUIRED NO_MODULE)

# IRRLICHT Visualization library
if(SEAHOWL_ENABLE_IRRLICHT)
    set(IRRLICHT_COMP Irrlicht)
    add_definitions("-DHAVE_IRRLICHT=1")
endif(SEAHOWL_ENABLE_IRRLICHT)

# Multy body dynamic library
find_package(
    Chrono
    COMPONENTS ${IRRLICHT_COMP}
    OPTIONAL_COMPONENTS PardisoMKL CONFIG
    REQUIRED
)

# JSON Parser library
find_package(nlohmann_json REQUIRED)

# AeroDyn module
if(SEAHOWL_ENABLE_AERODYN)
    find_package(AERODYN REQUIRED)
    add_compile_definitions(SEAHOWL_HAVE_AERODYN=1)
    add_definitions("-DHAVE_AERODYN=1")
endif(SEAHOWL_ENABLE_AERODYN)

# InflowWind module
if(SEAHOWL_ENABLE_INFLOWWIND)
    find_package(INFLOWWIND REQUIRED)
    add_compile_definitions(SEAHOWL_HAVE_INFLOWWIND=1)
    add_definitions("-DHAVE_INFLOWWIND=1")
endif(SEAHOWL_ENABLE_INFLOWWIND)

if(NOT CMAKE_SIZEOF_VOID_P EQUAL 8)
	message(FATAL_ERROR "ONLY 64 BITS SUPPORTED")
endif()

if(SEAHOWL_ENABLE_VTK)
    find_package(VTK 9 COMPONENTS CommonDataModel IOXML FiltersGeneral FiltersModeling FiltersSources FiltersExtraction)
endif(SEAHOWL_ENABLE_VTK)

if(VTK_FOUND)
    add_compile_definitions(SEAHOWL_HAVE_VTK=1)
    add_definitions("-DHAVE_VTK=1")
endif()


# python
if(SEAHOWL_ENABLE_PYTHON)
    find_package(pybind11 REQUIRED)
endif(SEAHOWL_ENABLE_PYTHON)

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
    include/seahowl/commons/numerics.h
    include/seahowl/commons/entities.h
    include/seahowl/commons/utils.h
)
source_group(commons FILES ${SEAHOWL_COMMONS_SOURCES} ${SEAHOWL_COMMONS_HEADERS})

# core
set(SEAHOWL_CORE_SOURCES
    src/core/reference_point.cpp
    src/core/rotor.cpp
    src/core/blade.cpp
    src/core/tower.cpp
    src/core/turbine.cpp
    src/core/system.cpp
)
set(SEAHOWL_CORE_HEADERS
    include/seahowl/core/reference_point.h
    include/seahowl/core/component.h
    include/seahowl/core/blade.h
    include/seahowl/core/rotor.h
    include/seahowl/core/tower.h
    include/seahowl/core/turbine.h
    include/seahowl/core/system.h
)
source_group(core FILES ${SEAHOWL_CORE_SOURCES} ${SEAHOWL_CORE_HEADERS})

# elasto
set(SEAHOWL_ELASTO_SOURCES
    src/elasto/component_elasto.cpp
    src/elasto/blade_elasto.cpp
    src/elasto/reference_point_elasto.cpp
    src/elasto/rotor_elasto.cpp
    src/elasto/tower_elasto.cpp
    src/elasto/mooring_elasto.cpp
    src/elasto/turbine_elasto.cpp
    src/elasto/chrono_adapters.cpp
)
set(SEAHOWL_ELASTO_HEADERS
    include/seahowl/elasto/blade_elasto.h
    include/seahowl/elasto/entities_elasto.h
    include/seahowl/elasto/component_elasto.h
    include/seahowl/elasto/system_elasto.h
    include/seahowl/elasto/reference_point_elasto.h
    include/seahowl/elasto/rotor_elasto.h
    include/seahowl/elasto/tower_elasto.h
    include/seahowl/elasto/mooring_elasto.h
    include/seahowl/elasto/turbine_elasto.h
    include/seahowl/elasto/chrono_adapters.h
)
source_group(elasto FILES ${SEAHOWL_ELASTO_SOURCES} ${SEAHOWL_ELASTO_HEADERS})

# aero
set(SEAHOWL_AERO_SOURCES
    src/aero/airfoil.cpp
    src/aero/bemt.cpp
    src/aero/reference_point_aero.cpp
    src/aero/blade_aero.cpp
    src/aero/rotor_aero.cpp
    src/aero/tower_aero.cpp
    src/aero/turbine_aero.cpp
    src/aero/wind_models.cpp
)
set(SEAHOWL_AERO_HEADERS
    include/seahowl/aero/airfoil.h
    include/seahowl/aero/bemt.h
    include/seahowl/aero/blade_aero.h
    include/seahowl/aero/reference_point_aero.h
    include/seahowl/aero/system_aero.h
    include/seahowl/aero/rotor_aero.h
    include/seahowl/aero/tower_aero.h
    include/seahowl/aero/turbine_aero.h
    include/seahowl/aero/wind_models.h
)
if (SEAHOWL_ENABLE_AERODYN)
    list(APPEND SEAHOWL_AERO_SOURCES src/aero/aerodyn_adapter.cpp)
    list(APPEND SEAHOWL_AERO_HEADERS include/seahowl/aero/aerodyn_adapter.h)
endif(SEAHOWL_ENABLE_AERODYN)
if (SEAHOWL_ENABLE_INFLOWWIND)
    list(APPEND SEAHOWL_AERO_SOURCES src/aero/inflowwind_adapter.cpp)
    list(APPEND SEAHOWL_AERO_HEADERS include/seahowl/aero/inflowwind_adapter.h)
endif(SEAHOWL_ENABLE_INFLOWWIND)
source_group(aero FILES
    ${SEAHOWL_AERO_SOURCES}
    ${SEAHOWL_AERO_HEADERS})

# servo
set(SEAHOWL_SERVO_SOURCES
    src/servo/controller.cpp
    src/servo/controller_discon.cpp
)
set(SEAHOWL_SERVO_HEADERS
    include/seahowl/servo/controller.h
    include/seahowl/servo/controller_discon.h
)
source_group(servo FILES ${SEAHOWL_SERVO_SOURCES} ${SEAHOWL_SERVO_HEADERS})

# io
set(SEAHOWL_IO_SOURCES src/io/read_json.cpp src/io/write_csv.cpp)
set(SEAHOWL_IO_HEADERS include/seahowl/io/read_json.h include/seahowl/io/write_csv.h)
if(SEAHOWL_ENABLE_VTK)
    list(APPEND SEAHOWL_IO_SOURCES src/io/write_vtk.cpp)
    list(APPEND SEAHOWL_IO_HEADERS include/seahowl/io/write_vtk.h)
endif(SEAHOWL_ENABLE_VTK)
if(SEAHOWL_ENABLE_IRRLICHT)
    list(APPEND SEAHOWL_IO_SOURCES src/io/viz_insitu.cpp)
    list(APPEND SEAHOWL_IO_HEADERS include/seahowl/io/viz_insitu.h)
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
    ${SEAHOWL_AERO_SOURCES}
    ${SEAHOWL_AERO_HEADERS}
    ${SEAHOWL_SERVO_SOURCES}
    ${SEAHOWL_SERVO_HEADERS}
    ${SEAHOWL_IO_SOURCES}
    ${SEAHOWL_IO_HEADERS}
)
target_sources(seahowl_core PRIVATE ${SEAHOWL_FILES})

target_include_directories(
    seahowl_core
    PUBLIC $<BUILD_INTERFACE:${PROJECT_SOURCE_DIR}/include> $<INSTALL_INTERFACE:include>
    PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/ ${CMAKE_CURRENT_SOURCE_DIR}/include ${CHRONO_INCLUDE_DIRS}
)

target_compile_features(seahowl_core PRIVATE cxx_std_17)

set_target_properties(
    seahowl_core
    PROPERTIES LINKER_LANGUAGE CXX
               CXX_EXTENSIONS OFF
               COMPILE_FLAGS "${CHRONO_CXX_FLAGS} ${EXTRA_COMPILE_FLAGS}"
               COMPILE_DEFINITIONS "CHRONO_DATA_DIR=\"${CHRONO_DATA_DIR}\""
               LINK_FLAGS "${CHRONO_LINKER_FLAGS}"
)

target_link_libraries(seahowl_core
    PUBLIC Eigen3::Eigen
    PRIVATE nlohmann_json::nlohmann_json ${CHRONO_LIBRARIES})

if(SEAHOWL_ENABLE_VTK)
    target_link_libraries(seahowl_core PRIVATE VTK::CommonDataModel VTK::IOXML)
endif(SEAHOWL_ENABLE_VTK)

if (SEAHOWL_ENABLE_AERODYN)
    target_link_libraries(seahowl_core PRIVATE AERODYN::AERODYN)
endif (SEAHOWL_ENABLE_AERODYN)

if (SEAHOWL_ENABLE_INFLOWWIND)
    target_link_libraries(seahowl_core PRIVATE INFLOWWIND::INFLOWWIND)
endif (SEAHOWL_ENABLE_INFLOWWIND)


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

set_target_properties(
    seahowl_driver
    PROPERTIES LINKER_LANGUAGE CXX
               CXX_EXTENSIONS OFF
               COMPILE_FLAGS "${CHRONO_CXX_FLAGS} ${EXTRA_COMPILE_FLAGS}"
               COMPILE_DEFINITIONS "CHRONO_DATA_DIR=\"${CHRONO_DATA_DIR}\""
               LINK_FLAGS "${CHRONO_LINKER_FLAGS}"
)

target_link_libraries(seahowl_driver PRIVATE seahowl_core nlohmann_json::nlohmann_json ${CHRONO_LIBRARIES})

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
# DOCUMENTATION
# ====================
if(SEAHOWL_ENABLE_DOC)
    add_subdirectory(doc)
endif(SEAHOWL_ENABLE_DOC)

# ====================
# PYTHON
# ====================
if (SEAHOWL_ENABLE_PYTHON)
    set(SEAHOWL_PYTHON_SOURCE_DIR ${CMAKE_CURRENT_SOURCE_DIR}/src/bindings)
    pybind11_add_module(pyseahowl ${SEAHOWL_PYTHON_SOURCE_DIR}/pybind_core.cpp)
    target_link_libraries(pyseahowl PRIVATE seahowl_core)
endif (SEAHOWL_ENABLE_PYTHON)
