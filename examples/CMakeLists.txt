
# ===================
# DISCON Example
# ===================
add_executable(ex_discon)

target_sources(ex_discon
    PRIVATE
    cpp/ex_discon.cpp
)

target_include_directories(ex_discon
    PRIVATE
	$<BUILD_INTERFACE:${PROJECT_SOURCE_DIR}/include>
	${CMAKE_CURRENT_SOURCE_DIR}/
	${CHRONO_INCLUDE_DIRS}
)

target_link_libraries(ex_discon
    PRIVATE
	seahowl_core
)


# ===================
# MOORING Example
# ===================
add_executable(ex_mooring)

target_sources(ex_mooring
    PRIVATE
    cpp/ex_mooring.cpp
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
	seahowl_core spdlog::spdlog
)
