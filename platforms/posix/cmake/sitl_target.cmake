set(SITL_WORKING_DIR ${PX4_BINARY_DIR}/tmp)
file(MAKE_DIRECTORY ${SITL_WORKING_DIR})
file(MAKE_DIRECTORY ${SITL_WORKING_DIR}/rootfs)

# add a symlink to the logs dir to make it easier to find them
add_custom_command(OUTPUT ${PX4_BINARY_DIR}/logs
	COMMAND ${CMAKE_COMMAND} -E create_symlink ${SITL_WORKING_DIR}/rootfs/log logs
	WORKING_DIRECTORY ${PX4_BINARY_DIR}
)
add_custom_target(logs_symlink DEPENDS ${PX4_BINARY_DIR}/logs)
add_dependencies(px4 logs_symlink)

add_custom_target(run_config
	COMMAND Tools/sitl_run.sh $<TARGET_FILE:px4> ${config_sitl_debugger} ${config_sitl_viewer} ${config_sitl_model} ${PX4_SOURCE_DIR} ${PX4_BINARY_DIR}
	WORKING_DIRECTORY ${SITL_WORKING_DIR}
	USES_TERMINAL
	DEPENDS px4 logs_symlink
)

px4_add_git_submodule(TARGET git_jmavsim PATH "${PX4_SOURCE_DIR}/Tools/jMAVSim")

# Add support for external project building
include(ExternalProject)

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

# project to build sitl_gazebo if necessary
px4_add_git_submodule(TARGET git_gazebo PATH "${PX4_SOURCE_DIR}/Tools/sitl_gazebo")
ExternalProject_Add(sitl_gazebo
	SOURCE_DIR ${PX4_SOURCE_DIR}/Tools/sitl_gazebo
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

px4_add_git_submodule(TARGET git_ign_gazebo PATH "${PX4_SOURCE_DIR}/Tools/simulation-ignition")
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

px4_add_git_submodule(TARGET git_flightgear_bridge PATH "${PX4_SOURCE_DIR}/Tools/flightgear_bridge")
ExternalProject_Add(flightgear_bridge
	SOURCE_DIR ${PX4_SOURCE_DIR}/Tools/flightgear_bridge
	CMAKE_ARGS -DCMAKE_INSTALL_PREFIX=${CMAKE_INSTALL_PREFIX}
	BINARY_DIR ${PX4_BINARY_DIR}/build_flightgear_bridge
	INSTALL_COMMAND ""
	DEPENDS git_flightgear_bridge
	USES_TERMINAL_CONFIGURE true
	USES_TERMINAL_BUILD true
	EXCLUDE_FROM_ALL true
	BUILD_ALWAYS 1
)

px4_add_git_submodule(TARGET git_jsbsim_bridge PATH "${PX4_SOURCE_DIR}/Tools/jsbsim_bridge")
ExternalProject_Add(jsbsim_bridge
	SOURCE_DIR ${PX4_SOURCE_DIR}/Tools/jsbsim_bridge
	CMAKE_ARGS -DCMAKE_INSTALL_PREFIX=${CMAKE_INSTALL_PREFIX}
	BINARY_DIR ${PX4_BINARY_DIR}/build_jsbsim_bridge
	INSTALL_COMMAND ""
	DEPENDS git_jsbsim_bridge
	USES_TERMINAL_CONFIGURE true
	USES_TERMINAL_BUILD true
	EXCLUDE_FROM_ALL true
	BUILD_ALWAYS 1
)

# create targets for each viewer/model/debugger combination
set(viewers
	none
	jmavsim
	gazebo
	ignition
)

set(debuggers
	none
	gdb
	lldb
	valgrind
	callgrind
)

set(models
	none
	believer
	boat
	cloudship
	glider
	if750a
	iris
	iris_ctrlalloc
	iris_dual_gps
	iris_foggy_lidar
	iris_irlock
	iris_obs_avoid
	iris_opt_flow
	iris_opt_flow_mockup
	iris_rplidar
	iris_vision
	nxp_cupcar
	plane
	plane_ctrlalloc
	plane_cam
	plane_catapult
	plane_lidar
	px4vision
	r1_rover
	rover
	shell
	solo
	standard_vtol
	standard_vtol_drop
	standard_vtol_ctrlalloc
	tailsitter
	techpod
	tiltrotor
	tiltrotor_ctrlalloc
	typhoon_h480
	typhoon_h480_ctrlalloc
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

set(all_posix_vmd_make_targets)
foreach(viewer ${viewers})
	foreach(debugger ${debuggers})
		foreach(model ${models})
			foreach(world ${worlds})
				if(world STREQUAL "none")
					if(debugger STREQUAL "none")
						if(model STREQUAL "none")
							set(_targ_name "${viewer}")
						else()
							set(_targ_name "${viewer}_${model}")
						endif()
					else()
						if(model STREQUAL "none")
							set(_targ_name "${viewer}___${debugger}")
						else()
							set(_targ_name "${viewer}_${model}_${debugger}")
						endif()
					endif()

					add_custom_target(${_targ_name}
						COMMAND ${PX4_SOURCE_DIR}/Tools/sitl_run.sh $<TARGET_FILE:px4> ${debugger} ${viewer} ${model} ${world} ${PX4_SOURCE_DIR} ${PX4_BINARY_DIR}
						WORKING_DIRECTORY ${SITL_WORKING_DIR}
						USES_TERMINAL
						DEPENDS logs_symlink
					)
					list(APPEND all_posix_vmd_make_targets ${_targ_name})
					if(viewer STREQUAL "gazebo")
						add_dependencies(${_targ_name} px4 sitl_gazebo)
					elseif(viewer STREQUAL "jmavsim")
						add_dependencies(${_targ_name} px4 git_jmavsim)
					elseif(viewer STREQUAL "ignition")
						add_dependencies(${_targ_name} px4 simulation-ignition)
					endif()
				else()
					if(viewer STREQUAL "gazebo")
						if(debugger STREQUAL "none")
							if(model STREQUAL "none")
								set(_targ_name "${viewer}___${world}")
							else()
								set(_targ_name "${viewer}_${model}__${world}")
							endif()
						else()
							if(model STREQUAL "none")
								set(_targ_name "${viewer}__${debugger}_${world}")
							else()
								set(_targ_name "${viewer}_${model}_${debugger}_${world}")
							endif()
						endif()

						add_custom_target(${_targ_name}
							COMMAND ${PX4_SOURCE_DIR}/Tools/sitl_run.sh $<TARGET_FILE:px4> ${debugger} ${viewer} ${model} ${world} ${PX4_SOURCE_DIR} ${PX4_BINARY_DIR}
							WORKING_DIRECTORY ${SITL_WORKING_DIR}
							USES_TERMINAL
							DEPENDS logs_symlink
						)
						list(APPEND all_posix_vmd_make_targets ${_targ_name})
						add_dependencies(${_targ_name} px4 sitl_gazebo)
					endif()
				endif()
			endforeach()
		endforeach()
	endforeach()
endforeach()

# create targets for jsbsim
set(models_jsbsim
	none
	rascal
	quadrotor_x
	hexarotor_x
	malolo
)

set(worlds_jsbsim
	none
	LSZH
)

foreach(debugger ${debuggers})
	foreach(model ${models_jsbsim})
		foreach(world ${worlds_jsbsim})
			if(world STREQUAL "none")
				if(debugger STREQUAL "none")
					if(model STREQUAL "none")
						set(_targ_name "jsbsim")
					else()
						set(_targ_name "jsbsim_${model}")
					endif()
				else()
					if(model STREQUAL "none")
						set(_targ_name "jsbsim__${debugger}_${world}")
					else()
						set(_targ_name "jsbsim_${model}_${debugger}_${world}")
					endif()
				endif()

				add_custom_target(${_targ_name}
					COMMAND ${PX4_SOURCE_DIR}/Tools/sitl_run.sh $<TARGET_FILE:px4> ${debugger} jsbsim ${model} "LSZH" ${PX4_SOURCE_DIR} ${PX4_BINARY_DIR}
					WORKING_DIRECTORY ${SITL_WORKING_DIR}
					USES_TERMINAL
					DEPENDS logs_symlink
				)
				list(APPEND all_posix_vmd_make_targets ${_targ_name})
				add_dependencies(${_targ_name} px4 jsbsim_bridge)
			else()
				if(debugger STREQUAL "none")
					if(model STREQUAL "none")
						set(_targ_name "jsbsim___${world}")
					else()
						set(_targ_name "jsbsim_${model}__${world}")
					endif()
				else()
					if(model STREQUAL "none")
						set(_targ_name "jsbsim___${debugger}_${world}")
					else()
						set(_targ_name "jsbsim_${model}_${debugger}_${world}")
					endif()
				endif()

				add_custom_target(${_targ_name}
					COMMAND ${PX4_SOURCE_DIR}/Tools/sitl_run.sh $<TARGET_FILE:px4> ${debugger} jsbsim ${model} ${world} ${PX4_SOURCE_DIR} ${PX4_BINARY_DIR}
					WORKING_DIRECTORY ${SITL_WORKING_DIR}
					USES_TERMINAL
					DEPENDS logs_symlink
				)
				list(APPEND all_posix_vmd_make_targets ${_targ_name})
				add_dependencies(${_targ_name} px4 jsbsim_bridge)
			endif()
		endforeach()
	endforeach()
endforeach()

# add flighgear targets
if(ENABLE_LOCKSTEP_SCHEDULER STREQUAL "no")
	set(models
		rascal
		rascal-electric
		tf-g1
		tf-r1
	)
	set(all_posix_vmd_make_targets)

	foreach(model ${models})
		set(_targ_name "flightgear_${model}")
		add_custom_target(${_targ_name}
			COMMAND ${PX4_SOURCE_DIR}/Tools/sitl_run.sh $<TARGET_FILE:px4> none flightgear ${model} none ${PX4_SOURCE_DIR} ${PX4_BINARY_DIR}
			WORKING_DIRECTORY ${SITL_WORKING_DIR}
			USES_TERMINAL
			DEPENDS logs_symlink
		)

		add_dependencies(${_targ_name} px4 flightgear_bridge)
		list(APPEND all_posix_vmd_make_targets ${_targ_name})
	endforeach()
endif()

string(REPLACE ";" "," posix_vmd_make_target_list "${all_posix_vmd_make_targets}")

add_custom_target(list_vmd_make_targets
	COMMAND sh -c "printf \"${posix_vmd_make_target_list}\\n\""
	COMMENT "List of acceptable '${PX4_BOARD}' <viewer_model_debugger> targets:"
	VERBATIM
	)
