
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

if (AVAILABLE_PHYSICAL_MEMORY)
	# Allow an additional job for every 1.5GB of available physical memory
	math(EXPR parallel_jobs "${AVAILABLE_PHYSICAL_MEMORY}/(3*1024/2)")
else()
	set(AVAILABLE_PHYSICAL_MEMORY "?")
endif()

if(parallel_jobs GREATER NUMBER_OF_LOGICAL_CORES)
	set(parallel_jobs ${NUMBER_OF_LOGICAL_CORES})
endif()

message(DEBUG  "${NUMBER_OF_LOGICAL_CORES} logical cores detected and ${AVAILABLE_PHYSICAL_MEMORY} megabytes of memory available.
		Limiting sitl_gazebo and simulation-ignition concurrent jobs to ${parallel_jobs}")

px4_add_git_submodule(TARGET git_ign_gazebo PATH "${PX4_SOURCE_DIR}/Tools/simulation-ignition")

include(ExternalProject)
ExternalProject_Add(simulation-ignition
	SOURCE_DIR ${PX4_SOURCE_DIR}/Tools/simulation-ignition
	CMAKE_ARGS -DCMAKE_INSTALL_PREFIX=${CMAKE_INSTALL_PREFIX}
	BINARY_DIR ${PX4_BINARY_DIR}/build_ign_gazebo
	INSTALL_COMMAND ""
	DEPENDS git_ign_gazebo
	USES_TERMINAL_CONFIGURE true
	USES_TERMINAL_BUILD true
	EXCLUDE_FROM_ALL true
	BUILD_ALWAYS 1
	BUILD_COMMAND ${CMAKE_COMMAND} --build <BINARY_DIR> -- -j ${parallel_jobs}
)

set(models
	iris
)

set(debugger none)
set(viewer ignition)
set(world iris)

foreach(model ${models})

	add_custom_target(ignition__${model}
		COMMAND
			${PX4_SOURCE_DIR}/Tools/sitl_run.sh $<TARGET_FILE:px4> ${debugger} ${viewer} ${model} ${world} ${PX4_SOURCE_DIR} ${PX4_BINARY_DIR}
		WORKING_DIRECTORY ${SITL_WORKING_DIR}
		USES_TERMINAL
		DEPENDS
			logs_symlink
			px4
			simulation-ignition
	)

endforeach()
