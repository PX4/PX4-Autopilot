
include_directories(${CMAKE_CURRENT_BINARY_DIR})

get_property(module_libraries GLOBAL PROPERTY PX4_MODULE_LIBRARIES)

# When building with catkin, do not change CMAKE_RUNTIME_OUTPUT_DIR from
# CMAKE_CURRENT_BINARY_DIR ''./platforms/posix' to './bin'
if (NOT CATKIN_DEVEL_PREFIX)
	set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/bin)
	set(CMAKE_RUNTIME_OUTPUT_DIRECTORY_DEBUG ${CMAKE_BINARY_DIR}/bin)
	set(CMAKE_RUNTIME_OUTPUT_DIRECTORY_RELEASE ${CMAKE_BINARY_DIR}/bin)
endif()

set(PX4_SHELL_COMMAND_PREFIX "px4-")

add_definitions("-DPX4_SHELL_COMMAND_PREFIX=\"${PX4_SHELL_COMMAND_PREFIX}\"")

px4_posix_generate_builtin_commands(
	OUT apps
	MODULE_LIST ${module_libraries})

px4_posix_generate_alias(
	OUT ${CMAKE_RUNTIME_OUTPUT_DIRECTORY}/px4-alias.sh
	MODULE_LIST ${module_libraries}
	PREFIX ${PX4_SHELL_COMMAND_PREFIX}
)

add_executable(px4
	src/px4/common/main.cpp
	apps.cpp
	)

target_link_libraries(px4
	PRIVATE
		${module_libraries}
		m

		# horrible circular dependencies that need to be teased apart
		px4_layer px4_platform
		work_queue
		parameters
)

if((NOT APPLE) AND (NOT ANDROID))
	target_link_libraries(px4 PRIVATE rt)
endif()

if(NOT ANDROID)
	target_link_libraries(px4 PRIVATE pthread)
endif()

target_link_libraries(px4 PRIVATE uORB)

#=============================================================================
# install
#

# TODO: extend to snapdragon

# px4 dirs
install(
	DIRECTORY
		${PROJECT_SOURCE_DIR}/posix-configs
		${PROJECT_SOURCE_DIR}/test
		${CMAKE_BINARY_DIR}/etc
		${CMAKE_RUNTIME_OUTPUT_DIRECTORY}
	DESTINATION
		${PROJECT_NAME}
	USE_SOURCE_PERMISSIONS
)

# Module Symlinks
px4_posix_generate_symlinks(
	MODULE_LIST ${module_libraries}
	PREFIX ${PX4_SHELL_COMMAND_PREFIX}
	TARGET px4
)

if (config_romfs_root)
	add_subdirectory(${PX4_SOURCE_DIR}/ROMFS ${PX4_BINARY_DIR}/ROMFS)
	add_dependencies(px4 romfs_gen_files_target)
endif()


# board defined upload helper
if(EXISTS "${PX4_BOARD_DIR}/cmake/upload.cmake")
	include(${PX4_BOARD_DIR}/cmake/upload.cmake)
endif()


if("${PX4_BOARD}" MATCHES "beaglebone_blue")
	target_link_libraries(px4 PRIVATE robotics_cape)

elseif("${PX4_BOARD}" MATCHES "emlid_navio2")
	target_link_libraries(px4 PRIVATE atomic)

	# vscode launch.json
	configure_file(${CMAKE_CURRENT_SOURCE_DIR}/Debug/launch_rpi.json.in ${PX4_SOURCE_DIR}/.vscode/launch.json COPYONLY)

elseif("${PX4_BOARD}" MATCHES "sitl")

	include(ExternalProject)
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

	# vscode launch.json
	if(${PX4_BOARD_LABEL} MATCHES "replay")
		configure_file(${CMAKE_CURRENT_SOURCE_DIR}/Debug/launch_replay.json.in ${PX4_SOURCE_DIR}/.vscode/launch.json COPYONLY)
	else()
		configure_file(${CMAKE_CURRENT_SOURCE_DIR}/Debug/launch_sim.json.in ${PX4_SOURCE_DIR}/.vscode/launch.json COPYONLY)
	endif()


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


	include(sitl_target)

	include(sitl_target_flightgear)
	include(sitl_target_gazebo)
	include(sitl_target_ignition)
	include(sitl_target_jmavsim)
	include(sitl_target_jsbsim)

	if(BUILD_TESTING)
		include(sitl_tests)
	endif()

	# install

	# px4 dirs
	install(
		DIRECTORY
			${PROJECT_SOURCE_DIR}/integrationtests
			${PROJECT_SOURCE_DIR}/launch
			${CMAKE_RUNTIME_OUTPUT_DIRECTORY}
		DESTINATION
			${PROJECT_NAME}
		USE_SOURCE_PERMISSIONS
		)

	# px4 files
	install(
		FILES
			${PROJECT_SOURCE_DIR}/CMakeLists.txt
			${PROJECT_SOURCE_DIR}/package.xml
		DESTINATION
			${PROJECT_NAME}
		)

	# px4 Tools dirs
	install(
		DIRECTORY
			${PROJECT_SOURCE_DIR}/Tools/ecl_ekf
		DESTINATION
			${PROJECT_NAME}/Tools
		USE_SOURCE_PERMISSIONS
		)

	# px4 Tools files
	install(
		PROGRAMS
			${PROJECT_SOURCE_DIR}/Tools/setup_gazebo.bash
			${PROJECT_SOURCE_DIR}/Tools/upload_log.py
		DESTINATION
			${PROJECT_NAME}/Tools
		)

	# ROMFS files
	install(
		DIRECTORY
			${PX4_BINARY_DIR}/etc
		DESTINATION
			${PROJECT_NAME}/build/px4_sitl_default
		)

	# sitl_gazebo built plugins
	install(
		DIRECTORY
			${PX4_BINARY_DIR}/build_gazebo
		DESTINATION
			${PROJECT_NAME}/build/px4_sitl_default
		FILES_MATCHING
			PATTERN "CMakeFiles" EXCLUDE
			PATTERN "*.so"
		)

	# sitl_gazebo dirs
	install(
		DIRECTORY
			${PROJECT_SOURCE_DIR}/Tools/sitl_gazebo/models
			${PROJECT_SOURCE_DIR}/Tools/sitl_gazebo/worlds
		DESTINATION
			${PROJECT_NAME}/Tools/sitl_gazebo
		)

	# sitl_gazebo files
	install(
		FILES
			${PROJECT_SOURCE_DIR}/Tools/sitl_gazebo/CMakeLists.txt
			${PROJECT_SOURCE_DIR}/Tools/sitl_gazebo/package.xml
		DESTINATION
			${PROJECT_NAME}/Tools/sitl_gazebo
		)

endif()
