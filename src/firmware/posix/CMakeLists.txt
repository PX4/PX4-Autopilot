include_directories(${CMAKE_CURRENT_BINARY_DIR})

px4_posix_generate_builtin_commands(
	OUT apps.h
	MODULE_LIST ${module_libraries})

if ("${BOARD}" STREQUAL "eagle" OR ("${BOARD}" STREQUAL "excelsior"))

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
			${PX4_SOURCE_DIR}/src/platforms/posix/main.cpp
			apps.h
		LINK_LIBS
			-Wl,--start-group
			${module_libraries}
			${df_driver_libs}
			${FASTRPC_ARM_LIBS}
			pthread m rt
			-Wl,--end-group
		)

	px4_add_adb_push(OUT upload
			 OS ${OS}
			 BOARD ${BOARD}
			 FILES ${CMAKE_CURRENT_BINARY_DIR}/px4
				 ${PX4_SOURCE_DIR}/posix-configs/eagle/flight/mainapp.config
			 DEPENDS px4
			 DEST /home/linaro)

elseif ("${BOARD}" STREQUAL "rpi")

	px4_add_executable(px4
		${PX4_SOURCE_DIR}/src/platforms/posix/main.cpp
		apps.h
		)

	target_link_libraries(px4
		-Wl,--start-group
		${module_libraries}
		df_driver_framework
		${df_driver_libs}
		pthread m rt
		-Wl,--end-group
		)

	px4_add_scp_push(OUT upload
			 OS ${OS}
			 BOARD ${BOARD}
			 FILES ${CMAKE_CURRENT_BINARY_DIR}/px4
				 ${PX4_SOURCE_DIR}/posix-configs/rpi/px4.config
			 DEPENDS px4
			 DEST /home/pi)

elseif ("${BOARD}" STREQUAL "bebop")

	set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -static")
	set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -static")

	px4_add_executable(px4
		${PX4_SOURCE_DIR}/src/platforms/posix/main.cpp
		apps.h
		)

	if (NOT APPLE)
		target_link_libraries(px4
			-Wl,--start-group
			${module_libraries}
			${df_driver_libs}
			pthread m rt
			-Wl,--end-group
			)
	else()
		target_link_libraries(px4
			${module_libraries}
			${df_driver_libs}
			pthread m
			)
	endif()

	px4_add_adb_push_to_bebop(OUT upload
			 OS ${OS}
			 BOARD ${BOARD}
			 FILES ${CMAKE_CURRENT_BINARY_DIR}/px4
			 DEPENDS px4
			 DEST /usr/bin)

else()

	px4_add_executable(px4
		${PX4_SOURCE_DIR}/src/platforms/posix/main.cpp
		apps.h
		)

	if (NOT APPLE)
		target_link_libraries(px4
			-Wl,--start-group
			${module_libraries}
			${df_driver_libs}
			pthread m rt
			-Wl,--end-group
			)
	else()
		target_link_libraries(px4
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

# project to build sitl_gazebo if necessary
ExternalProject_Add(sitl_gazebo
	SOURCE_DIR ${PX4_SOURCE_DIR}/Tools/sitl_gazebo
	CMAKE_ARGS -DCMAKE_INSTALL_PREFIX=${CMAKE_INSTALL_PREFIX}
	BINARY_DIR ${PX4_BINARY_DIR}/build_gazebo
	INSTALL_COMMAND ""
	)
set_target_properties(sitl_gazebo PROPERTIES EXCLUDE_FROM_ALL TRUE)

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
				add_dependencies(${_targ_name} sitl_gazebo)
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

#=============================================================================
# tests
#

add_test(NAME rcS_tests
	COMMAND ${PX4_SOURCE_DIR}/Tools/sitl_run.sh
		$<TARGET_FILE:px4>
		posix-configs/SITL/init/test
		none
		none
		none
		${PX4_SOURCE_DIR}
		${PX4_BINARY_DIR}
		WORKING_DIRECTORY ${SITL_WORKING_DIR})
set_tests_properties(rcS_tests PROPERTIES
	PASS_REGULAR_EXPRESSION "All tests passed")

# vim: set noet ft=cmake fenc=utf-8 ff=unix :
