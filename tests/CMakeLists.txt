#  GTest may need pthreads
set(THREADS_PREFER_PTHREAD_FLAG ON)
find_package(Threads REQUIRED)

find_package(googletest CONFIG REQUIRED)

add_executable(test_01 unit_tests/test_components.cpp )

set_target_properties(test_01 

	PROPERTIES

		CXX_EXTENSIONS OFF
		COMPILE_FLAGS "${CHRONO_CXX_FLAGS} ${EXTRA_COMPILE_FLAGS}"
		COMPILE_DEFINITIONS "CHRONO_DATA_DIR=\"${CHRONO_DATA_DIR}\""
		LINK_FLAGS "${CHRONO_LINKER_FLAGS}"
)

target_compile_features(
	test_01

	PRIVATE
	cxx_std_17
)


# target_link_libraries(test_01 seahowl_core ${CHRONO_LIBRARIES} -lgtest)

target_link_libraries(test_01 seahowl_core ${CHRONO_LIBRARIES} gtest_main)

target_include_directories(test_01
	PRIVATE
	${PROJECT_SOURCE_DIR}
	$<TARGET_PROPERTY:seahowl_core,INCLUDE_DIRECTORIES>)

add_test(
	NAME test_01 
	COMMAND $<TARGET_FILE:test_01> ${SEAHOWL_DATADIR}
)

