cmake_minimum_required(VERSION 3.10)

set_property(GLOBAL PROPERTY USE_FOLDERS ON)

# ==================
# SEAHOWL PROJECT
# =================
project(
    SEAHOWL
    VERSION 0.0.2
    DESCRIPTION "SEAHOWL: Servo-Elasto-Aero-Hydro Offsore Wind Laboratory"
    LANGUAGES CXX)

# guard against in-source builds
if(${CMAKE_SOURCE_DIR} STREQUAL ${CMAKE_BINARY_DIR})
    message(
        FATAL_ERROR
            "In-source builds not allowed. Please make a new directory (called a build directory) and run CMake from there. You may need to remove CMakeCache.txt."
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
option(SEAHOWL_ENABLE_ROSCO "Enable ROSCO controller" ON)
option(SEAHOWL_ENABLE_VTK "Enable VTK Library for output" OFF)
option(SEAHOWL_FETCH_DEPENDENCIES "Fetch dependencies" OFF)

# =======================
# External libraries
# =======================

# Irrlicht (in-situ visualization)
if(SEAHOWL_ENABLE_IRRLICHT)
    set(IRRLICHT_COMP Irrlicht)
    add_definitions("-DHAVE_IRRLICHT=1")
endif(SEAHOWL_ENABLE_IRRLICHT)

# Chrono
find_package(
    Chrono
    COMPONENTS ${IRRLICHT_COMP}
    OPTIONAL_COMPONENTS PardisoMKL CONFIG
    REQUIRED)

# json
if(SEAHOWL_FETCH_DEPENDENCIES)
    include(FetchContent)
    FetchContent_Declare(
        nlohmann_json
        GIT_REPOSITORY https://github.com/nlohmann/json
        GIT_TAG v3.11.2
        GIT_PROGRESS TRUE
        OVERRIDE_FIND_PACKAGE)
endif(SEAHOWL_FETCH_DEPENDENCIES)
find_package(nlohmann_json REQUIRED)

# python
if(SEAHOWL_ENABLE_PYTHON)
    if(SEAHOWL_FETCH_DEPENDENCIES)
        FetchContent_Declare(
            pybind11
            GIT_REPOSITORY https://github.com/pybind/pybind11
            GIT_TAG        v2.10.0
            GIT_PROGRESS TRUE
            OVERRIDE_FIND_PACKAGE
        )
    endif(SEAHOWL_FETCH_DEPENDENCIES)
    find_package(pybind11 CONFIG REQUIRED)
endif(SEAHOWL_ENABLE_PYTHON)

# tests
if(SEAHOWL_ENABLE_TESTS)
    if(SEAHOWL_FETCH_DEPENDENCIES)
        FetchContent_Declare(
            googletest
            GIT_REPOSITORY https://github.com/google/googletest
            GIT_TAG        release-1.12.1
            GIT_PROGRESS TRUE
            OVERRIDE_FIND_PACKAGE)
    endif(SEAHOWL_FETCH_DEPENDENCIES)
    find_package(googletest CONFIG REQUIRED)
    # For Windows: Prevent overriding the parent project's compiler/linker settings
    set(gtest_force_shared_crt ON CACHE BOOL "" FORCE)
endif(SEAHOWL_ENABLE_TESTS)

#VTK
if(SEAHOWL_ENABLE_VTK)
    if(SEAHOWL_FETCH_DEPENDENCIES)
        FetchContent_Declare(
            VTK
            GIT_REPOSITORY https://gitlab.kitware.com/vtk/vtk.git
            GIT_TAG        v9.2.0
            GIT_PROGRESS TRUE
            OVERRIDE_FIND_PACKAGE)
    endif(SEAHOWL_FETCH_DEPENDENCIES)
    find_package(VTK 9
        COMPONENTS
        CommonDataModel
        IOXML
        FiltersGeneral
        FiltersModeling
        FiltersSources
        FiltersExtraction)
    add_compile_definitions(SEAHOWL_HAVE_VTK=1)
    add_definitions("-DHAVE_VTK=1")
endif(SEAHOWL_ENABLE_VTK)

# ROSCO controller
if(SEAHOWL_ENABLE_ROSCO)
    find_package(ROSCO REQUIRED)
    add_compile_definitions(SEAHOWL_HAVE_ROSCO=1)
    add_definitions("-DHAVE_ROSCO=1")
endif(SEAHOWL_ENABLE_ROSCO)

if(NOT CMAKE_SIZEOF_VOID_P EQUAL 8)
    message(FATAL_ERROR "ONLY 64 BITS SUPPORTED")
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
    if(${MSVC_VERSION} GREATER_EQUAL 1915) # Fix for VS 2017 15.8 and newer to
                                           # handle alignment specification with
                                           # Eigen
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
# If the directory is not set as environment variable set a default value in
# CMake variable. This default value can be modified in the CMakeCache.txt file
# in build directory.
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

set(SEAHOWL_SOURCES
    # core
    src/core/reference_point.cpp
    src/core/rotor.cpp
    src/core/blade.cpp
    src/core/tower.cpp
    src/core/turbine.cpp
    src/core/utils.cpp
    src/core/system.cpp
    # aero
    src/aero/airfoil.cpp
    src/aero/bemt.cpp
    src/aero/reference_point_aero.cpp
    src/aero/blade_aero.cpp
    src/aero/rotor_aero.cpp
    src/aero/tower_aero.cpp
    src/aero/wind_models.cpp
    # servo
    src/servo/controller.cpp
    # elasto
    src/elasto/elasto.cpp
    src/elasto/blade_elasto.cpp
    src/elasto/reference_point_elasto.cpp
    src/elasto/rotor_elasto.cpp
    src/elasto/tower_elasto.cpp
    # io
    src/io/read_json.cpp
    src/io/write_csv.cpp)

set(SEAHOWL_HEADERS
    # core
    include/seahowl/core/reference_point.h
    include/seahowl/core/blade.h
    include/seahowl/core/rotor.h
    include/seahowl/core/tower.h
    include/seahowl/core/turbine.h
    include/seahowl/core/utils.h
    include/seahowl/core/system.h
    # aero
    include/seahowl/aero/airfoil.h
    include/seahowl/aero/bemt.h
    include/seahowl/aero/blade_aero.h
    include/seahowl/aero/reference_point_aero.h
    include/seahowl/aero/rotor_aero.h
    include/seahowl/aero/tower_aero.h
    include/seahowl/aero/wind_models.h
    # servo
    include/seahowl/servo/controller.h
    # elasto
    include/seahowl/elasto/blade_elasto.h
    include/seahowl/elasto/elasto.h
    include/seahowl/elasto/reference_point_elasto.h
    include/seahowl/elasto/rotor_elasto.h
    include/seahowl/elasto/tower_elasto.h
    include/seahowl/elasto/utils_elasto.h
    # io
    include/seahowl/io/read_json.h
    include/seahowl/io/write_csv.h)

if(SEAHOWL_ENABLE_ROSCO)
    list(APPEND SEAHOWL_HEADERS include/seahowl/servo/controller_discon.h)
    list(APPEND SEAHOWL_SOURCES src/servo/controller_discon.cpp)
endif(SEAHOWL_ENABLE_ROSCO)

if(SEAHOWL_ENABLE_VTK)
    list(APPEND SEAHOWL_HEADERS include/seahowl/io/write_vtk.h)
    list(APPEND SEAHOWL_SOURCES src/io/write_vtk.cpp)
endif(SEAHOWL_ENABLE_VTK)

target_sources(seahowl_core PRIVATE ${SEAHOWL_SOURCES} ${SEAHOWL_HEADERS})

target_include_directories(
    seahowl_core
    PUBLIC $<BUILD_INTERFACE:${PROJECT_SOURCE_DIR}/include>
           $<INSTALL_INTERFACE:include> ${CHRONO_INCLUDE_DIRS}
    PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/ ${CMAKE_CURRENT_SOURCE_DIR}/include)

target_compile_features(seahowl_core PRIVATE cxx_std_17)

set_target_properties(
    seahowl_core
    PROPERTIES LINKER_LANGUAGE CXX
               CXX_EXTENSIONS OFF
               COMPILE_FLAGS "${CHRONO_CXX_FLAGS} ${EXTRA_COMPILE_FLAGS}"
               COMPILE_DEFINITIONS "CHRONO_DATA_DIR=\"${CHRONO_DATA_DIR}\""
               LINK_FLAGS "${CHRONO_LINKER_FLAGS}")

target_link_libraries(seahowl_core PRIVATE nlohmann_json::nlohmann_json
                                           ${CHRONO_LIBRARIES})

if(SEAHOWL_ENABLE_ROSCO)
    target_link_libraries(seahowl_core PRIVATE ROSCO::ROSCO)
endif(SEAHOWL_ENABLE_ROSCO)

if(SEAHOWL_ENABLE_VTK)
    target_link_libraries(seahowl_core PRIVATE VTK::CommonDataModel VTK::IOXML)
endif(SEAHOWL_ENABLE_VTK)

if (SEAHOWL_ENABLE_PYTHON)
    set(SEAHOWL_PYTHON_SOURCE_DIR ${CMAKE_CURRENT_SOURCE_DIR}/src/python)
    pybind11_add_module(pyseahowl ${SEAHOWL_PYTHON_SOURCE_DIR}/pybindings.cpp)
    target_link_libraries(pyseahowl PUBLIC seahowl_core)
endif (SEAHOWL_ENABLE_PYTHON)


# =============
# DRIVER
# =============
add_executable(seahowl_driver)

target_sources(
    seahowl_driver PRIVATE src/main.cpp
                           # ${SEAHOWL_HEADERS}
)

target_include_directories(
    seahowl_driver
    PUBLIC $<BUILD_INTERFACE:${PROJECT_SOURCE_DIR}/include>
           $<INSTALL_INTERFACE:include>
    PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/ ${CMAKE_CURRENT_SOURCE_DIR}/include
            ${CHRONO_INCLUDE_DIRS})

target_compile_features(seahowl_driver PRIVATE cxx_std_17)

set_target_properties(
    seahowl_driver
    PROPERTIES LINKER_LANGUAGE CXX
               CXX_EXTENSIONS OFF
               COMPILE_FLAGS "${CHRONO_CXX_FLAGS} ${EXTRA_COMPILE_FLAGS}"
               COMPILE_DEFINITIONS "CHRONO_DATA_DIR=\"${CHRONO_DATA_DIR}\""
               LINK_FLAGS "${CHRONO_LINKER_FLAGS}")

target_link_libraries(
    seahowl_driver PRIVATE seahowl_core nlohmann_json::nlohmann_json
                           ${CHRONO_LIBRARIES})

if(SEAHOWL_ENABLE_ROSCO)
    target_link_libraries(seahowl_driver PRIVATE ROSCO::ROSCO)
endif(SEAHOWL_ENABLE_ROSCO)

if(SEAHOWL_ENABLE_VTK)
    target_link_libraries(seahowl_driver PRIVATE VTK::CommonDataModel
                                                 VTK::IOXML)
endif(SEAHOWL_ENABLE_VTK)

# --------------------------------------------------------------
# === 4 (OPTIONAL) ===
#
# Optionally, add a custom command for copying all Chrono and dependency DLLs to
# the appropriate binary output folder. This function has effect only on
# Windows.
#
# Note that you must first set EXECUTABLE_OUTPUT_PATH (this can simply be
# ${CMAKE_BINARY_DIR}, like in this example)
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
