include_directories(${CMAKE_CURRENT_BINARY_DIR})

px4_posix_generate_builtin_commands(
	OUT apps.h
	MODULE_LIST ${module_libraries})

if ("${BOARD}" STREQUAL "eagle")
	FASTRPC_STUB_GEN(../qurt/px4muorb.idl)

	set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -Wno-missing-prototypes -Wno-missing-declarations")
	QURT_APP(
		APP_NAME mainapp
		IDL_NAME px4muorb
		APPS_DEST "/home/linaro"
		SOURCES
		px4muorb_stub.c
		${CMAKE_SOURCE_DIR}/src/platforms/posix/main.cpp
		apps.h
		LINK_LIBS
		-Wl,--start-group
		${module_libraries}
		df_driver_framework
		${df_driver_libs}
		${FASTRPC_ARM_LIBS}
		pthread m rt
		-Wl,--end-group
		)

	px4_add_adb_push(OUT upload
			 OS ${OS}
			 BOARD ${BOARD}
			 FILES ${CMAKE_CURRENT_BINARY_DIR}/mainapp
				 ${CMAKE_SOURCE_DIR}/posix-configs/eagle/flight/mainapp.config
			 DEPENDS build_mainapp_apps
			 DEST /home/linaro)

else()
	add_executable(mainapp
		${CMAKE_SOURCE_DIR}/src/platforms/posix/main.cpp
		apps.h
		)
	if (NOT APPLE)
		target_link_libraries(mainapp
			-Wl,--start-group
			${module_libraries}
			df_driver_framework
			${df_driver_libs}
			pthread m rt
			-Wl,--end-group
			)
	else()
		target_link_libraries(mainapp
			${module_libraries}
			df_driver_framework
			${df_driver_libs}
			pthread m
			)
	endif()
endif()

add_custom_target(run_config
	COMMAND Tools/sitl_run.sh "${config_sitl_rcS}" "${config_sitl_debugger}"
	"${config_sitl_viewer}" "${config_sitl_model}" "${CMAKE_BINARY_DIR}"
	WORKING_DIRECTORY ${CMAKE_SOURCE_DIR}
	USES_TERMINAL
	)
add_dependencies(run_config mainapp)

foreach(viewer none jmavsim gazebo replay)
	foreach(debugger none gdb lldb ddd valgrind)
		foreach(model none iris iris_opt_flow tailsitter standard_vtol plane)
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
				COMMAND Tools/sitl_run.sh "${config_sitl_rcS}"
				"${debugger}"
				"${viewer}" "${model}" "${CMAKE_BINARY_DIR}"
				WORKING_DIRECTORY ${CMAKE_SOURCE_DIR}
				USES_TERMINAL
				)
			add_dependencies(${_targ_name} mainapp)
		endforeach()
	endforeach()
endforeach()

# vim: set noet ft=cmake fenc=utf-8 ff=unix :
