#  GTest may need pthreads
set(THREADS_PREFER_PTHREAD_FLAG ON)
find_package(Threads REQUIRED)

find_package(GTest CONFIG REQUIRED)

# set default SEAHOWL_TESTDIR
set(SEAHOWL_TESTDIR_VALUE ${CMAKE_CURRENT_LIST_DIR})
message(STATUS "SEAHOWL_TESTDIR set to: ${SEAHOWL_TESTDIR_VALUE}")
add_definitions(-DSEAHOWL_TESTDIR="${SEAHOWL_TESTDIR_VALUE}")

set(TEST_FILES
	unit_tests/test_bemt.cpp
	unit_tests/test_blade.cpp
	unit_tests/test_components.cpp
	unit_tests/test_entities.cpp
	unit_tests/test_rotor.cpp
	unit_tests/test_tower.cpp
	unit_tests/test_turbine.cpp
	unit_tests/test_controller.cpp
	unit_tests/test_morison.cpp
	unit_tests/tools/test_framework_dataset.cpp
	unit_tests/tools/csv.cpp
	unit_tests/tools/get_env_var.cpp
)

if(SEAHOWL_ENABLE_AERODYN)
    list(APPEND TEST_FILES unit_tests/test_aerodyn.cpp)
endif(SEAHOWL_ENABLE_AERODYN)

if(SEAHOWL_ENABLE_INFLOWWIND)
    list(APPEND TEST_FILES unit_tests/test_inflowwind.cpp)
endif(SEAHOWL_ENABLE_INFLOWWIND)

add_executable(test_components ${TEST_FILES} )

target_include_directories(test_components
    PRIVATE
	$<BUILD_INTERFACE:${PROJECT_SOURCE_DIR}/include>
	${CMAKE_CURRENT_SOURCE_DIR}/
	${CHRONO_INCLUDE_DIRS}
)

target_compile_features(
    test_components
    PRIVATE cxx_std_17
)

target_compile_options(
    test_components
    PRIVATE ${CHRONO_CXX_FLAGS}
)

target_link_libraries(test_components
    PRIVATE
	seahowl_core
	GTest::gtest GTest::gtest_main GTest::gmock_main GTest::gmock Threads::Threads spdlog::spdlog
)

add_test(
	NAME test_components
	COMMAND $<TARGET_FILE:test_components> ${SEAHOWL_DATADIR}
)
