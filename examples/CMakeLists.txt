
if (SEAHOWL_ENABLE_ROSCO)


add_executable( ex_rosco
)

target_sources(ex_rosco

    PRIVATE
    ex_rosco.cpp
)

target_include_directories(ex_rosco


	PUBLIC
	$<BUILD_INTERFACE:${PROJECT_SOURCE_DIR}/include>
	$<INSTALL_INTERFACE:include>

    PRIVATE
	${CMAKE_CURRENT_SOURCE_DIR}/
	${CMAKE_PROJECT_SOURCE_DIR}
)

target_compile_features(
	ex_rosco

	PRIVATE
	cxx_std_17
)

set_target_properties(ex_rosco

    PROPERTIES
        FOLDER examples
        LINKER_LANGUAGE CXX
        CXX_EXTENSIONS OFF
	    COMPILE_FLAGS "${CHRONO_CXX_FLAGS} ${EXTRA_COMPILE_FLAGS}"
	    COMPILE_DEFINITIONS "CHRONO_DATA_DIR=\"${CHRONO_DATA_DIR}\""
	    LINK_FLAGS "${CHRONO_LINKER_FLAGS}"
)


target_link_libraries(ex_rosco
    PRIVATE
	seahowl_core
)


endif (SEAHOWL_ENABLE_ROSCO)

# ===================
# BLADE Example
# ===================
add_executable( ex_blade )

target_sources(ex_blade

    PRIVATE
    ex_blade.cpp
)

target_include_directories(ex_blade

    PRIVATE
	$<BUILD_INTERFACE:${PROJECT_SOURCE_DIR}/include>
	${CMAKE_CURRENT_SOURCE_DIR}/
	${CHRONO_INCLUDE_DIRS}

)

target_compile_features(
	ex_blade

	PRIVATE
	cxx_std_17
)

set_target_properties(ex_blade

    PROPERTIES
        FOLDER examples
        LINKER_LANGUAGE CXX
        CXX_EXTENSIONS OFF
	    COMPILE_FLAGS "${CHRONO_CXX_FLAGS} ${EXTRA_COMPILE_FLAGS}"
	    COMPILE_DEFINITIONS "CHRONO_DATA_DIR=\"${CHRONO_DATA_DIR}\""
	    LINK_FLAGS "${CHRONO_LINKER_FLAGS}"
)

message("CHRONO ${CHRONO_DATA_DIR}")
target_link_libraries(ex_blade
    PRIVATE
	seahowl_core
)


# ===================
# MOORING Example
# ===================
add_executable( ex_mooring )

target_sources(ex_mooring
    PRIVATE
    ex_mooring.cpp
)

target_include_directories(ex_mooring
    PRIVATE
	$<BUILD_INTERFACE:${PROJECT_SOURCE_DIR}/include>
	${CMAKE_CURRENT_SOURCE_DIR}/
	${CHRONO_INCLUDE_DIRS}
)

target_compile_features(
	ex_mooring
	PRIVATE
	cxx_std_17
)

set_target_properties(ex_mooring
    PROPERTIES
        FOLDER examples
        LINKER_LANGUAGE CXX
        CXX_EXTENSIONS OFF
	    COMPILE_FLAGS "${CHRONO_CXX_FLAGS} ${EXTRA_COMPILE_FLAGS}"
	    COMPILE_DEFINITIONS "CHRONO_DATA_DIR=\"${CHRONO_DATA_DIR}\""
	    LINK_FLAGS "${CHRONO_LINKER_FLAGS}"
)

message("CHRONO ${CHRONO_DATA_DIR}")
target_link_libraries(ex_mooring
    PRIVATE
	seahowl_core
)
