#  GTest may need pthreads
set(THREADS_PREFER_PTHREAD_FLAG ON)
find_package(Threads REQUIRED)

find_package(GTest CONFIG REQUIRED)

add_executable(test_01 unit_tests/test_components.cpp )

target_include_directories(test_01
    PRIVATE
	$<BUILD_INTERFACE:${PROJECT_SOURCE_DIR}/include>
	${CMAKE_CURRENT_SOURCE_DIR}/
	${CHRONO_INCLUDE_DIRS}
)

target_compile_features(
    test_01
    PRIVATE cxx_std_17
)

target_compile_options(
    test_01
    PRIVATE ${CHRONO_CXX_FLAGS}
)

target_link_libraries(test_01
    PRIVATE
	seahowl_core
	GTest::gtest GTest::gtest_main GTest::gmock_main GTest::gmock Threads::Threads
)

add_test(
	NAME test_01
	COMMAND $<TARGET_FILE:test_01> ${SEAHOWL_DATADIR}
)
