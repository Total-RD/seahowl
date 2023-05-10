
if (SEAHOWL_ENABLE_ROSCO)

add_executable(ex_rosco)

target_sources(ex_rosco
    PRIVATE
    ex_rosco.cpp
)

target_include_directories(ex_rosco
    PRIVATE
	$<BUILD_INTERFACE:${PROJECT_SOURCE_DIR}/include>
	${CMAKE_CURRENT_SOURCE_DIR}/
	${CHRONO_INCLUDE_DIRS}
)

target_link_libraries(ex_rosco
    PRIVATE
	seahowl_core
)

endif (SEAHOWL_ENABLE_ROSCO)

# ===================
# BLADE Example
# ===================
add_executable(ex_blade)

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

target_compile_features(ex_mooring
    PRIVATE
    cxx_std_17
)

target_link_libraries(ex_mooring
    PRIVATE
	seahowl_core
)
