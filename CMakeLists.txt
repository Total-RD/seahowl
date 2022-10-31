cmake_minimum_required(VERSION 3.10)

set_property(GLOBAL PROPERTY USE_FOLDERS ON)


# ==================
# SEAHOWL PROJECT
# =================
project( SEAHOWL 
         VERSION 0.0.2
         DESCRIPTION "Wind Turbine simulation with CHRONO"
         LANGUAGES CXX
)

# guard against in-source builds
if(${CMAKE_SOURCE_DIR} STREQUAL ${CMAKE_BINARY_DIR})
    message(FATAL_ERROR "In-source builds not allowed. Please make a new directory (called a build directory) and run CMake from there. You may need to remove CMakeCache.txt. ")
endif()


LIST(APPEND CMAKE_PREFIX_PATH "${CMAKE_INSTALL_PREFIX}/../Chrono/lib")  # TO BE CLEANDED

# Add the cmake folder so the FindSphinx module is found
set(CMAKE_MODULE_PATH "${PROJECT_SOURCE_DIR}/cmake" ${CMAKE_MODULE_PATH})

# =============
# OPTIONS
# =============

option (SEAHOWL_ENABLE_IRRLICHT "Enable Irrlicht 3D visualization library" OFF)
option (SEAHOWL_ENABLE_TESTS "Enable tests" ON)
option (SEAHOWL_ENABLE_DOC "Generate html documentation" ON)
option (SEAHOWL_ENABLE_BUILD "Build library and drivers" ON)
option (SEAHOWL_ENABLE_PYTHON "Enable python binding" ON)
option (SEAHOWL_ENABLE_EXAMPLES "Enable examples" ON)
option (SEAHOWL_ENABLE_ROSCO "Enable ROSCO controller" ON)
option (SEAHOWL_ENABLE_VTK "Enable VTK Library for output" ON)

# =======================
# External libraries
# =======================

# IRRLICHT Visualization library
if(SEAHOWL_ENABLE_IRRLICHT)
    set(IRRLICHT_COMP Irrlicht )
    add_definitions("-DHAVE_IRRLICHT=1")
endif(SEAHOWL_ENABLE_IRRLICHT)

# Multy body dynamic library
find_package(Chrono
            COMPONENTS ${IRRLICHT_COMP}
            OPTIONAL_COMPONENTS PardisoMKL
            CONFIG REQUIRED)

# JSON Parser library
find_package(nlohmann_json REQUIRED)

# ROSCO Discon Wind Turbine controller
if (SEAHOWL_ENABLE_ROSCO)
    find_package(ROSCO REQUIRED)
    add_compile_definitions(SEAHOWL_HAVE_ROSCO=1)
    add_definitions("-DHAVE_ROSCO=1")
endif( SEAHOWL_ENABLE_ROSCO)


if( NOT CMAKE_SIZEOF_VOID_P EQUAL 8 )
	message(FATAL_ERROR "ONLY 64 BITS SUPPORTED")
endif()

if (SEAHOWL_ENABLE_VTK)
	find_package(VTK 9

		COMPONENTS
		CommonDataModel
		IOXML
		FiltersGeneral
		FiltersModeling
		FiltersSources
		FiltersExtraction

	)
endif (SEAHOWL_ENABLE_VTK)

if(VTK_FOUND)
    add_compile_definitions(SEAHOWL_HAVE_VTK=1)
    add_definitions("-DHAVE_VTK=1")
endif()

#--------------------------------------------------------------
# Tweaks to disable some warnings with MSVC
#--------------------------------------------------------------
if(MSVC)
    add_definitions("-D_CRT_SECURE_NO_DEPRECATE")  # avoids deprecation warnings
    add_definitions("-D_SCL_SECURE_NO_DEPRECATE")  # avoids deprecation warnings
    add_definitions( "-DNOMINMAX" )                # do not use MSVC's min/max macros
    add_compile_options(/wd4275)                   # disable warnings triggered by Irrlicht
    add_compile_options(/wd4251)                   # class needs to have dll-interface

    ## TO BE REMOVED: SHOULD BE ALREADY SET BY CHRONO LIBRARY
    if(${MSVC_VERSION} GREATER_EQUAL 1915)         # Fix for VS 2017 15.8 and newer to handle alignment specification with Eigen
        add_definitions( "-D_ENABLE_EXTENDED_ALIGNED_STORAGE" )
    endif()
else()
    set(EXTRA_COMPILE_FLAGS "")
endif()

# ==============
# DATADIR
# ==============
# Define a data directory to find test input files
#
# If the directory is not set as environment variable 
# set a default value in CMake variable. This default
# value can be modified in the CMakeCache.txt file
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
add_library( seahowl_core )



set(SEAHOWL_SOURCES 

    src/elasto/elasto.cpp
    src/elasto/blade_elasto.cpp
    src/elasto/reference_point_elasto.cpp
    src/elasto/rotor_elasto.cpp
    src/elasto/tower_elasto.cpp
    src/elasto/mooring_elasto.cpp

    src/core/reference_point.cpp
    src/core/rotor.cpp
    src/core/blade.cpp
    src/core/tower.cpp
    src/core/turbine.cpp
    src/core/utils.cpp 
    src/core/system.cpp

    src/aero/airfoil.cpp
    src/aero/bemt.cpp
    src/aero/reference_point_aero.cpp
    src/aero/blade_aero.cpp
    src/aero/rotor_aero.cpp
    src/aero/tower_aero.cpp
    src/aero/wind_models.cpp

    src/servo/controller.cpp

    src/io/read_json.cpp
    src/io/write_csv.cpp
)

set(SEAHOWL_HEADERS

    include/seahowl/core/reference_point.h
    include/seahowl/core/blade.h
    include/seahowl/core/rotor.h
    include/seahowl/core/tower.h
    include/seahowl/core/turbine.h
    include/seahowl/core/utils.h
    include/seahowl/core/system.h


    include/seahowl/aero/airfoil.h
    include/seahowl/aero/bemt.h
    include/seahowl/aero/blade_aero.h
    include/seahowl/aero/reference_point_aero.h
    include/seahowl/aero/rotor_aero.h
    include/seahowl/aero/tower_aero.h
    include/seahowl/aero/wind_models.h

    include/seahowl/servo/controller.h

    include/seahowl/elasto/blade_elasto.h
    include/seahowl/elasto/elasto.h
    include/seahowl/elasto/reference_point_elasto.h
    include/seahowl/elasto/rotor_elasto.h
    include/seahowl/elasto/tower_elasto.h
    include/seahowl/elasto/utils_elasto.h
    include/seahowl/elasto/mooring_elasto.h

    include/seahowl/io/read_json.h
    include/seahowl/io/write_csv.h

)

if (SEAHOWL_ENABLE_ROSCO)
    list(APPEND SEAHOWL_HEADERS 
        include/seahowl/servo/controller_discon.h
    )
    list(APPEND SEAHOWL_SOURCES 
        src/servo/controller_discon.cpp
    )
endif (SEAHOWL_ENABLE_ROSCO)

if (SEAHOWL_ENABLE_VTK)
    list(APPEND SEAHOWL_HEADERS
        include/seahowl/io/write_vtk.h
    )
    list(APPEND SEAHOWL_SOURCES
        src/io/write_vtk.cpp
    )
endif (SEAHOWL_ENABLE_VTK)

target_sources(seahowl_core 
    PRIVATE 
    ${SEAHOWL_SOURCES}
    ${SEAHOWL_HEADERS}
)

target_include_directories(seahowl_core

	PUBLIC
	$<BUILD_INTERFACE:${PROJECT_SOURCE_DIR}/include>
	$<INSTALL_INTERFACE:include>
    ${CHRONO_INCLUDE_DIRS}

	PRIVATE
	${CMAKE_CURRENT_SOURCE_DIR}/
	${CMAKE_CURRENT_SOURCE_DIR}/include
)

target_compile_features(
	seahowl_core

	PRIVATE
	cxx_std_17
)

set_target_properties(seahowl_core

    PROPERTIES
        LINKER_LANGUAGE CXX
        CXX_EXTENSIONS OFF
	    COMPILE_FLAGS "${CHRONO_CXX_FLAGS} ${EXTRA_COMPILE_FLAGS}"
	    COMPILE_DEFINITIONS "CHRONO_DATA_DIR=\"${CHRONO_DATA_DIR}\""
	    LINK_FLAGS "${CHRONO_LINKER_FLAGS}"
)


target_link_libraries(seahowl_core
    PRIVATE 
    nlohmann_json::nlohmann_json
    ${CHRONO_LIBRARIES}
)

if (SEAHOWL_ENABLE_ROSCO)
target_link_libraries(seahowl_core
    PRIVATE
    ROSCO::ROSCO
)
endif (SEAHOWL_ENABLE_ROSCO)

if(SEAHOWL_ENABLE_VTK) 
    target_link_libraries(seahowl_core
        PUBLIC
        VTK::CommonDataModel
        VTK::IOXML
    )
endif(SEAHOWL_ENABLE_VTK) 

# =============
# DRIVER 
# =============
add_executable( seahowl_driver )


target_sources(seahowl_driver 
    PRIVATE 
    src/main.cpp

    #${SEAHOWL_HEADERS}
)

target_include_directories(seahowl_driver

	PUBLIC
	$<BUILD_INTERFACE:${PROJECT_SOURCE_DIR}/include>
	$<INSTALL_INTERFACE:include>

    PRIVATE
	${CMAKE_CURRENT_SOURCE_DIR}/
	${CMAKE_CURRENT_SOURCE_DIR}/include
    ${CHRONO_INCLUDE_DIRS}
)

target_compile_features(
	seahowl_driver

	PRIVATE
	cxx_std_17
)

set_target_properties(seahowl_driver 

    PROPERTIES
        LINKER_LANGUAGE CXX
        CXX_EXTENSIONS OFF
	    COMPILE_FLAGS "${CHRONO_CXX_FLAGS} ${EXTRA_COMPILE_FLAGS}"
	    COMPILE_DEFINITIONS "CHRONO_DATA_DIR=\"${CHRONO_DATA_DIR}\""
	    LINK_FLAGS "${CHRONO_LINKER_FLAGS}"
)


target_link_libraries(seahowl_driver 
    PRIVATE 
    seahowl_core
    nlohmann_json::nlohmann_json
    ${CHRONO_LIBRARIES}
)

#--------------------------------------------------------------
# === 4 (OPTIONAL) ===
# 
# Optionally, add a custom command for copying all Chrono and
# dependency DLLs to the appropriate binary output folder.
# This function has effect only on Windows.
# 
# Note that you must first set EXECUTABLE_OUTPUT_PATH
# (this can simply be ${CMAKE_BINARY_DIR}, like in this example)
#--------------------------------------------------------------

set(EXECUTABLE_OUTPUT_PATH ${CMAKE_BINARY_DIR})
add_DLL_copy_command("${CHRONO_DLLS}")

# ============
# Examples
# ===========
if( SEAHOWL_ENABLE_EXAMPLES )
    add_subdirectory(examples)
endif( SEAHOWL_ENABLE_EXAMPLES )

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
