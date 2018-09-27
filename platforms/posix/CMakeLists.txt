
add_subdirectory(src)

include_directories(${CMAKE_CURRENT_BINARY_DIR})

get_property(module_libraries GLOBAL PROPERTY PX4_MODULE_LIBRARIES)

set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/bin)

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

if (("${BOARD}" STREQUAL "eagle") OR ("${BOARD}" STREQUAL "excelsior"))
	include(fastrpc)
	include(linux_app)

	FASTRPC_STUB_GEN(../qurt/px4muorb.idl)

	set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -Wno-missing-prototypes -Wno-missing-declarations")
	LINUX_APP(
		APP_NAME px4
		IDL_NAME px4muorb
		APPS_DEST "/home/linaro"
		SOURCES
			px4muorb_stub.c
			src/main.cpp
			apps.cpp
		LINK_LIBS
			-Wl,--start-group
				px4_layer
				parameters
				${module_libraries}
				${df_driver_libs}
				${FASTRPC_ARM_LIBS}
				pthread m rt
			-Wl,--end-group
		)

	add_custom_target(upload
		COMMAND ${PX4_SOURCE_DIR}/Tools/adb_upload.sh ${CMAKE_RUNTIME_OUTPUT_DIRECTORY} ${PX4_SOURCE_DIR}/posix-configs/eagle/flight/mainapp.config /home/linaro
		DEPENDS px4
		COMMENT "uploading px4"
		USES_TERMINAL
		)

else()
	px4_add_executable(px4
		src/main.cpp
		apps.cpp
		)

	target_link_libraries(px4
		PRIVATE
			${module_libraries}
			df_driver_framework
			${df_driver_libs}
			pthread m
	)

	if (NOT APPLE)
		target_link_libraries(px4 PRIVATE rt)
	endif()

	target_link_libraries(px4 PRIVATE modules__uORB)

	#=============================================================================
	# install
	#

	# TODO: extend to snapdragon

	# px4 dirs
	install(
		DIRECTORY
			${PROJECT_SOURCE_DIR}/posix-configs
			${PROJECT_SOURCE_DIR}/ROMFS
			${PROJECT_SOURCE_DIR}/test
			${CMAKE_RUNTIME_OUTPUT_DIRECTORY}
		DESTINATION
			${PROJECT_NAME}
		USE_SOURCE_PERMISSIONS
		)

endif()

# Module Symlinks
px4_posix_generate_symlinks(
	MODULE_LIST ${module_libraries}
	PREFIX ${PX4_SHELL_COMMAND_PREFIX}
	TARGET px4
)

if ("${BOARD}" STREQUAL "rpi")
	add_custom_target(upload
		COMMAND rsync -arh --progress ${CMAKE_RUNTIME_OUTPUT_DIRECTORY} ${PX4_SOURCE_DIR}/posix-configs/rpi/*.config ${PX4_SOURCE_DIR}/ROMFS pi@"$ENV{AUTOPILOT_HOST}":/home/pi
		DEPENDS px4
		COMMENT "uploading px4"
		USES_TERMINAL
		)

elseif ("${BOARD}" STREQUAL "bbblue")
	target_link_libraries(px4 PRIVATE robotcontrol)

	add_custom_target(upload
		COMMAND rsync -arh --progress ${CMAKE_RUNTIME_OUTPUT_DIRECTORY} ${PX4_SOURCE_DIR}/posix-configs/bbblue/*.config ${PX4_SOURCE_DIR}/ROMFS debian@BBBluePX4:/home/debian/px4
		DEPENDS px4
		COMMENT "uploading px4 and data files"
		USES_TERMINAL
		)

elseif ("${BOARD}" STREQUAL "bebop")

	add_custom_target(upload
		COMMAND ${PX4_SOURCE_DIR}/Tools/adb_upload_to_bebop.sh ${CMAKE_RUNTIME_OUTPUT_DIRECTORY}/. /data/ftp/internal_000
		DEPENDS px4
		COMMENT "uploading px4"
		USES_TERMINAL
		)

elseif ("${BOARD}" STREQUAL "sitl")

	include(sitl_target)
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

	# sitl_gazebo built plugins
	install(
		DIRECTORY
			${PROJECT_SOURCE_DIR}/build/posix_sitl_default/build_gazebo
		DESTINATION
			${PROJECT_NAME}/build/posix_sitl_default
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

elseif (("${BOARD}" STREQUAL "eagle") OR ("${BOARD}" STREQUAL "excelsior"))

	# package all libs?
	#install(
	#	FILES
	#		$ENV{HEXAGON_ARM_SYSROOT}/usr/lib/libadsprpc.so
	#	DESTINATION
	#		lib
	#	)

endif()
