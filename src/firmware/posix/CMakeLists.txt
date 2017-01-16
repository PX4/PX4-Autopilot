include_directories(${CMAKE_CURRENT_BINARY_DIR})

px4_posix_generate_builtin_commands(
	OUT apps
	MODULE_LIST ${module_libraries})

# Define build target
set(APP_NAME px4)
set(MAIN_SRC ${PX4_SOURCE_DIR}/src/platforms/posix/main.cpp)
set(UPLOAD_NAME upload)

if ("${BOARD}" STREQUAL "eagle" OR ("${BOARD}" STREQUAL "excelsior"))

	include(fastrpc)
	include(linux_app)

	FASTRPC_STUB_GEN(../qurt/px4muorb.idl)

	set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -Wno-missing-prototypes -Wno-missing-declarations")
	LINUX_APP(
		APP_NAME ${APP_NAME}
		IDL_NAME px4muorb
		APPS_DEST "/home/linaro"
		SOURCES
			px4muorb_stub.c
			${MAIN_SRC}
			apps.cpp
		LINK_LIBS
			-Wl,--start-group
			${module_libraries}
			${df_driver_libs}
			${FASTRPC_ARM_LIBS}
			pthread m rt
			-Wl,--end-group
		)

	px4_add_adb_push(OUT ${UPLOAD_NAME}
				OS ${OS}
				BOARD ${BOARD}
				FILES ${CMAKE_CURRENT_BINARY_DIR}/${APP_NAME}
				${PX4_SOURCE_DIR}/posix-configs/eagle/flight/mainapp.config
				DEPENDS ${APP_NAME}
				DEST /home/linaro)

elseif ("${BOARD}" STREQUAL "rpi")

	px4_add_executable(${APP_NAME}
		${MAIN_SRC}
		apps.cpp
		)

	target_link_libraries(${APP_NAME}
		-Wl,--start-group
		${module_libraries}
		df_driver_framework
		${df_driver_libs}
		pthread m rt
		-Wl,--end-group
		)

	file(GLOB RPI_CONFIG_FILES ${PX4_SOURCE_DIR}/posix-configs/rpi/*.config)
	px4_add_scp_push(OUT ${UPLOAD_NAME}
				OS ${OS}
				BOARD ${BOARD}
				FILES ${CMAKE_CURRENT_BINARY_DIR}/${APP_NAME}
				${RPI_CONFIG_FILES}
				${PX4_SOURCE_DIR}/ROMFS
				DEPENDS ${APP_NAME}
				DEST /home/pi)

elseif ("${BOARD}" STREQUAL "bebop")

	set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -static")
	set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -static")

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

	px4_add_adb_push_to_bebop(OUT ${UPLOAD_NAME}
				OS ${OS}
				BOARD ${BOARD}
				FILES ${CMAKE_CURRENT_BINARY_DIR}/${APP_NAME}
				DEPENDS ${APP_NAME}
				DEST /usr/bin)

elseif ("${BOARD}" STREQUAL "sitl")

	include(./sitl_target.cmake)
	include(./sitl_tests.cmake)

else()

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
endif()

#=============================================================================
# sitl run targets
#

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
ExternalProject_Add(optical_flow
	SOURCE_DIR ${PX4_SOURCE_DIR}/Tools/OpticalFlow/
	CMAKE_ARGS -DCMAKE_INSTALL_PREFIX=${CMAKE_INSTALL_PREFIX}
	BINARY_DIR ${PX4_BINARY_DIR}/build_optical_flow
	INSTALL_COMMAND ""
	)

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
	CMAKE_ARGS -DCMAKE_INSTALL_PREFIX=${CMAKE_INSTALL_PREFIX} -DNO_ROS=TRUE -DMAVLINK_INTERFACE=TRUE -DBUILD_OPTICAL_FLOW_PLUGIN=TRUE
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

# create targets for each viewer/model/debugger combination
set(viewers none jmavsim gazebo replay)
set(debuggers none gdb lldb ddd valgrind)
set(models none iris iris_opt_flow tailsitter standard_vtol plane solo typhoon_h480)
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
				# Add both the rotors_simulator/rotors_gazebo_plugins (plugin files) and
				# rotors_simulator/rotors_gazebo (world and model files) as dependencies
				add_dependencies(${_targ_name} rotors_simulator_rotors_gazebo_plugins rotors_simulator_rotors_gazebo)
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

#=============================================================================
# install
#

install(TARGETS px4 DESTINATION ${CMAKE_INSTALL_BINDIR})
install(DIRECTORY ${PROJECT_SOURCE_DIR}/ROMFS DESTINATION ${CMAKE_INSTALL_DATADIR}/${PROJECT_NAME})
install(DIRECTORY ${PROJECT_SOURCE_DIR}/posix-configs DESTINATION ${CMAKE_INSTALL_DATADIR}/${PROJECT_NAME})

# vim: set noet ft=cmake fenc=utf-8 ff=unix :
