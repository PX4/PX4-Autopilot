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

message(DEBUG "${NUMBER_OF_LOGICAL_CORES} logical cores detected and ${AVAILABLE_PHYSICAL_MEMORY} megabytes of memory available.
		Limiting sitl_gazebo concurrent jobs to ${parallel_jobs}")

# project to build sitl_gazebo if necessary
px4_add_git_submodule(TARGET git_gazebo PATH "${PX4_SOURCE_DIR}/Tools/simulation/gazebo/sitl_gazebo")
include(ExternalProject)
ExternalProject_Add(sitl_gazebo
	SOURCE_DIR ${PX4_SOURCE_DIR}/Tools/simulation/gazebo/sitl_gazebo
	CMAKE_ARGS
		-DCMAKE_INSTALL_PREFIX=${CMAKE_INSTALL_PREFIX}
		-DSEND_ODOMETRY_DATA=ON
		-DGENERATE_ROS_MODELS=ON
	BINARY_DIR ${PX4_BINARY_DIR}/build_gazebo
	INSTALL_COMMAND ""
	DEPENDS git_gazebo
	USES_TERMINAL_CONFIGURE true
	USES_TERMINAL_BUILD true
	EXCLUDE_FROM_ALL true
	BUILD_ALWAYS 1
	BUILD_COMMAND ${CMAKE_COMMAND} --build <BINARY_DIR> -- -j ${parallel_jobs}
)

# create targets for each model/world/debugger combination

set(debuggers
	none
	gdb
	lldb
	valgrind
	callgrind
)

set(models
	none
	advanced_plane
	believer
	boat
	cloudship
	glider
	iris
	iris_dual_gps
	iris_foggy_lidar
	iris_irlock
	iris_obs_avoid
	iris_opt_flow
	iris_opt_flow_mockup
	iris_rplidar
	iris_vision
	nxp_cupcar
	omnicopter
	plane
	plane_cam
	plane_catapult
	plane_lidar
	px4vision
	r1_rover
	rover
	shell
	standard_vtol
	standard_vtol_drop
	tailsitter
	techpod
	tiltrotor
	typhoon_h480
	uuv_bluerov2_heavy
	uuv_hippocampus
)

set(worlds
	none
	baylands
	empty
	ksql_airport
	mcmillan_airfield
	sonoma_raceway
	warehouse
	windy
	yosemite
)

foreach(debugger ${debuggers})
	foreach(model ${models})
		foreach(world ${worlds})
			if(world STREQUAL "none")
				if(debugger STREQUAL "none")
					if(model STREQUAL "none")
						set(_targ_name "gazebo")
					else()
						set(_targ_name "gazebo_${model}")
					endif()
				else()
					if(model STREQUAL "none")
						set(_targ_name "gazebo___${debugger}")
					else()
						set(_targ_name "gazebo_${model}_${debugger}")
					endif()
				endif()

				add_custom_target(${_targ_name}
					COMMAND ${PX4_SOURCE_DIR}/Tools/simulation/gazebo/sitl_run.sh $<TARGET_FILE:px4> ${debugger} ${model} ${world} ${PX4_SOURCE_DIR} ${PX4_BINARY_DIR}
					WORKING_DIRECTORY ${SITL_WORKING_DIR}
					USES_TERMINAL
					DEPENDS px4 sitl_gazebo
				)
			else()
				if(debugger STREQUAL "none")
					if(model STREQUAL "none")
						set(_targ_name "gazebo___${world}")
					else()
						set(_targ_name "gazebo_${model}__${world}")
					endif()
				else()
					if(model STREQUAL "none")
						set(_targ_name "gazebo__${debugger}_${world}")
					else()
						set(_targ_name "gazebo_${model}_${debugger}_${world}")
					endif()
				endif()

				add_custom_target(${_targ_name}
					COMMAND ${PX4_SOURCE_DIR}/Tools/simulation/gazebo/sitl_run.sh $<TARGET_FILE:px4> ${debugger} ${model} ${world} ${PX4_SOURCE_DIR} ${PX4_BINARY_DIR}
					WORKING_DIRECTORY ${SITL_WORKING_DIR}
					USES_TERMINAL
					DEPENDS px4 sitl_gazebo
				)
			endif()
		endforeach()
	endforeach()
endforeach()

# mavsdk tests currently depend on sitl_gazebo
ExternalProject_Add(mavsdk_tests
	SOURCE_DIR ${PX4_SOURCE_DIR}/test/mavsdk_tests
	CMAKE_ARGS -DCMAKE_INSTALL_PREFIX=${CMAKE_INSTALL_PREFIX}
	BINARY_DIR ${PX4_BINARY_DIR}/mavsdk_tests
	INSTALL_COMMAND ""
	USES_TERMINAL_CONFIGURE true
	USES_TERMINAL_BUILD true
	EXCLUDE_FROM_ALL true
	BUILD_ALWAYS 1
)
