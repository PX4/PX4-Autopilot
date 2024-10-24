set(GAZEBO_VERSION 8)

find_package(gz-sim8 QUIET)

if(gz-sim8_FOUND)

	# Estimate an appropriate number of parallel jobs
	cmake_host_system_information(RESULT AVAILABLE_PHYSICAL_MEMORY QUERY AVAILABLE_PHYSICAL_MEMORY)
	cmake_host_system_information(RESULT NUMBER_OF_LOGICAL_CORES QUERY NUMBER_OF_LOGICAL_CORES)

	set(parallel_jobs 1)

	if(NOT NUMBER_OF_LOGICAL_CORES)
		include(ProcessorCount)
		ProcessorCount(NUMBER_OF_LOGICAL_CORES)
	endif()

	if(NOT AVAILABLE_PHYSICAL_MEMORY AND NUMBER_OF_LOGICAL_CORES GREATER_EQUAL 4)
		# Memory estimate unavailable, use N-2 jobs
		math(EXPR parallel_jobs "${NUMBER_OF_LOGICAL_CORES} - 2")
	endif()

	if(AVAILABLE_PHYSICAL_MEMORY)
		# Allow an additional job for every 1.5GB of available physical memory
		math(EXPR parallel_jobs "${AVAILABLE_PHYSICAL_MEMORY}/(3*1024/2)")
	else()
		set(AVAILABLE_PHYSICAL_MEMORY "?")
	endif()

	if(parallel_jobs GREATER NUMBER_OF_LOGICAL_CORES)
		set(parallel_jobs ${NUMBER_OF_LOGICAL_CORES})
	endif()

	if(parallel_jobs LESS 1)
		set(parallel_jobs 1)
	endif()

	include(ExternalProject)

	ExternalProject_Add(gzsim-plugins
		SOURCE_DIR ${PX4_SOURCE_DIR}/Tools/simulation/gz/plugins/px4-gzsim-plugins/
		BINARY_DIR ${PX4_BINARY_DIR}/build_gz-sim_plugins
		INSTALL_COMMAND ""
		DEPENDS mavlink_c_generate
		USES_TERMINAL_CONFIGURE true
		USES_TERMINAL_BUILD true
		EXCLUDE_FROM_ALL true
		BUILD_ALWAYS 1
		BUILD_COMMAND ${CMAKE_COMMAND} --build <BINARY_DIR> -- -j ${parallel_jobs}
	)

	ExternalProject_Add(mavsdk_tests
		SOURCE_DIR ${PX4_SOURCE_DIR}/test/mavsdk_tests
		CMAKE_ARGS -DCMAKE_INSTALL_PREFIX=${CMAKE_INSTALL_PREFIX}
		BINARY_DIR ${PX4_BINARY_DIR}/mavsdk_tests
		DEPENDS mavlink_c_generate
		INSTALL_COMMAND ""
		USES_TERMINAL_CONFIGURE true
		USES_TERMINAL_BUILD true
		EXCLUDE_FROM_ALL true
		BUILD_ALWAYS 1
	)
endif()

