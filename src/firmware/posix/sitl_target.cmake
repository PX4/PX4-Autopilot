
function(px4_add_sitl_app)
px4_parse_function_args(
			NAME px4_add_sitl_app
			ONE_VALUE APP_NAME MAIN_SRC UPLOAD_NAME
			REQUIRED APP_NAME MAIN_SRC
			ARGN ${ARGN}
			)

	px4_add_executable(${APP_NAME}
			${MAIN_SRC}
			apps.cpp
			)

	if (NOT APPLE)
		target_link_libraries(${APP_NAME}
			-Wl,--start-group
			${module_libraries}
			${df_driver_libs}
			pthread m rt
			-Wl,--end-group
			)
	else()
		target_link_libraries(${APP_NAME}
			${module_libraries}
			${df_driver_libs}
			pthread m
			)
	endif()

endfunction()

#=============================================================================
# sitl run targets
#

set(SITL_RUNNER_MAIN_CPP ${PX4_SOURCE_DIR}/src/platforms/posix/main.cpp)
px4_add_sitl_app(APP_NAME px4
		UPLOAD_NAME upload
		MAIN_SRC ${SITL_RUNNER_MAIN_CPP}
		)

set(SITL_WORKING_DIR ${PX4_BINARY_DIR}/tmp)
file(MAKE_DIRECTORY ${SITL_WORKING_DIR})

add_custom_target(run_config
		COMMAND Tools/sitl_run.sh
			$<TARGET_FILE:px4>
			${config_sitl_rcS_dir}
			${config_sitl_debugger}
			${config_sitl_viewer}
			${config_sitl_model}
			${PX4_SOURCE_DIR}
			${PX4_BINARY_DIR}
			WORKING_DIRECTORY ${SITL_WORKING_DIR}
			USES_TERMINAL
		)
add_dependencies(run_config px4)

# Add OpticalFlow as an external project
#message(STATUS "CMAKE_INSTALL_PREFIX = ${CMAKE_INSTALL_PREFIX}")
ExternalProject_Add(optical_flow
	SOURCE_DIR ${PX4_SOURCE_DIR}/Tools/OpticalFlow/
	CMAKE_CACHE_ARGS -DCMAKE_INSTALL_PREFIX:PATH=${PX4_BINARY_DIR}/install_optical_flow
	BINARY_DIR ${PX4_BINARY_DIR}/build_optical_flow
	INSTALL_DIR ${PX4_BINARY_DIR}/install_optical_flow
	)
set_target_properties(optical_flow PROPERTIES EXCLUDE_FROM_ALL TRUE)

# Append build dir to CMAKE_MODULE_PATH, so find_package(OpticalFlow) can find FindOpticalFlow.cmake
message(STATUS "Appending ${PX4_BINARY_DIR}/build_optical_flow to CMAKE_MODULE_PATH.")
set(CMAKE_MODULE_PATH "${CMAKE_MODULE_PATH}" "${PX4_BINARY_DIR}/build_optical_flow")
message(STATUS "Appending ${PX4_BINARY_DIR}/build_optical_flow to CMAKE_PREFIX_PATH.")
set(CMAKE_PREFIX_PATH "${CMAKE_PREFIX_PATH}" "${PX4_BINARY_DIR}/build_optical_flow")

# Add rotors_simulator/rotors_gazebo_plugins as a "external project" (part of the rotors_simulator repo)
# NOTE: Changed from targetting sitl_gazebo to packages within the rotors_simulator repo
# sitl_gazebo had a global CMakeLists.txt which built everything, while
# rotors_simulator has many discreet catkin packages, each with it's own
# CMakeLists.txt
ExternalProject_Add(rotors_simulator_rotors_gazebo_plugins
	SOURCE_DIR ${PX4_SOURCE_DIR}/Tools/rotors_simulator/rotors_gazebo_plugins/
	# NO_ROS argument will build rotors_gazebo_plugins without any ROS dependencies
	# MAVLINK_INTERFACE will build with gazebo_mavlink_interface
	# BUILD_OPTICAL_FLOW_PLUGIN will build the optical flow plugin
	CMAKE_ARGS -DCMAKE_INSTALL_PREFIX:PATH=${CMAKE_INSTALL_PREFIX} -DNO_ROS=TRUE -DBUILD_MAVLINK_INTERFACE_PLUGIN=TRUE -DBUILD_OPTICAL_FLOW_PLUGIN=TRUE
	CMAKE_CACHE_ARGS -DCMAKE_MODULE_PATH:PATH=${CMAKE_MODULE_PATH} -DCMAKE_PREFIX_PATH:PATH=${CMAKE_PREFIX_PATH} -DADDITIONAL_INCLUDE_DIRS:PATH=${PX4_SOURCE_DIR}/Tools/mav_comm/mav_msgs/include/ -DMAVLINK_HEADER_DIR:PATH=${PX4_SOURCE_DIR}/mavlink/include/mavlink/v1.0
	BINARY_DIR ${PX4_BINARY_DIR}/build_gazebo
	INSTALL_COMMAND ""
	)
# When rotors_simulator_rotors_gazebo_plugins is passed BUILD_OPTICAL_FLOW_PLUGIN=TRUE is depends on the OpticalFlow library.
add_dependencies(rotors_simulator_rotors_gazebo_plugins optical_flow)
set_target_properties(rotors_simulator_rotors_gazebo_plugins PROPERTIES EXCLUDE_FROM_ALL TRUE)

# Add rotors_simulator/rotors_gazebo as a "external project" (part of the rotors_simulator repo)
ExternalProject_Add(rotors_simulator_rotors_gazebo
	SOURCE_DIR ${PX4_SOURCE_DIR}/Tools/rotors_simulator/rotors_gazebo/
	# NO_ROS argument will build rotors_gazebo without any ROS dependencies
	CMAKE_ARGS -DCMAKE_INSTALL_PREFIX=${CMAKE_INSTALL_PREFIX} -DNO_ROS=TRUE
	BINARY_DIR ${PX4_BINARY_DIR}/build_test
	INSTALL_COMMAND ""
	)
set_target_properties(rotors_simulator_rotors_gazebo PROPERTIES EXCLUDE_FROM_ALL TRUE)

ExternalProject_Add_Step(sitl_gazebo forceconfigure
	DEPENDEES update
	DEPENDERS configure
	ALWAYS 1)

# create targets for each viewer/model/debugger combination
set(viewers none jmavsim gazebo replay)
set(debuggers none ide gdb lldb ddd valgrind callgrind)
set(models none iris iris_opt_flow standard_vtol plane solo tailsitter typhoon_h480)
set(all_posix_vmd_make_targets)
foreach(viewer ${viewers})
	foreach(debugger ${debuggers})
		foreach(model ${models})
			if (debugger STREQUAL "none")
				if (model STREQUAL "none")
					set(_targ_name "${viewer}")
				else()
					set(_targ_name "${viewer}_${model}")
				endif()
			else()
				if (model STREQUAL "none")
					set(_targ_name "${viewer}___${debugger}")
				else()
					set(_targ_name "${viewer}_${model}_${debugger}")
				endif()
			endif()

			if (debugger STREQUAL "ide" AND viewer STREQUAL "gazebo")
				set(SITL_RUNNER_SOURCE_DIR ${PX4_SOURCE_DIR})
				set(SITL_RUNNER_MODEL_FILE ${PX4_SOURCE_DIR}/${config_sitl_rcS_dir}/${model})
				set(SITL_RUNNER_WORKING_DIRECTORY ${SITL_WORKING_DIR})

				configure_file(${PX4_SOURCE_DIR}/src/platforms/posix/sitl_runner_main.cpp.in sitl_runner_main_${model}.cpp @ONLY)

				px4_add_sitl_app(APP_NAME px4_${model}
						UPLOAD_NAME upload_${model}
						MAIN_SRC ${CMAKE_CURRENT_BINARY_DIR}/sitl_runner_main_${model}.cpp
						)
				set_target_properties(px4_${model} PROPERTIES EXCLUDE_FROM_ALL TRUE)
			endif()

			add_custom_target(${_targ_name}
					COMMAND ${PX4_SOURCE_DIR}/Tools/sitl_run.sh
						$<TARGET_FILE:px4>
						${config_sitl_rcS_dir}
						${debugger}
						${viewer}
						${model}
						${PX4_SOURCE_DIR}
						${PX4_BINARY_DIR}
						WORKING_DIRECTORY ${SITL_WORKING_DIR}
						USES_TERMINAL
					)
			list(APPEND all_posix_vmd_make_targets ${_targ_name})
			if (viewer STREQUAL "gazebo")
				add_dependencies(${_targ_name} rotors_simulator_rotors_gazebo rotors_simulator_rotors_gazebo_plugins)
				if (viewer STREQUAL "gazebo")
					add_dependencies(${_targ_name} px4_${model})
				endif()
			endif()
		endforeach()
	endforeach()
endforeach()

px4_join(OUT posix_vmd_make_target_list LIST ${all_posix_vmd_make_targets} GLUE "\\n")
add_custom_target(list_vmd_make_targets
	COMMAND sh -c "printf \"${posix_vmd_make_target_list}\\n\""
	COMMENT "List of acceptable '${CONFIG}' <viewer_model_debugger> targets:"
	VERBATIM
	)
