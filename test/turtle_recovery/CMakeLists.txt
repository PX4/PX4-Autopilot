# SPDX-License-Identifier: BSD-3-Clause

find_package(Threads REQUIRED)

foreach(suite rc core fault_matrix mapping interlock lifecycle)
	set(test_name unit-turtle_recovery_${suite})
	add_executable(${test_name} EXCLUDE_FROM_ALL test_${suite}.cpp)

	if(suite STREQUAL "interlock")
		target_sources(${test_name} PRIVATE interlock_other.cpp)
	endif()

	target_include_directories(${test_name} PRIVATE
		${PX4_SOURCE_DIR}/src
		${PX4_SOURCE_DIR}/src/drivers/dshot
	)
	# Keep assert() active in release builds.
	target_compile_options(${test_name} PRIVATE -UNDEBUG)
	target_link_libraries(${test_name} PRIVATE Threads::Threads)
	add_test(NAME ${test_name} COMMAND ${test_name})
	add_filtered_test_dependencies(${test_name})
endforeach()
