
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

# project to build sitl_gazebo if necessary
ExternalProject_Add(sitl_gazebo
	SOURCE_DIR ${PX4_SOURCE_DIR}/Tools/sitl_gazebo
	CMAKE_ARGS -DCMAKE_INSTALL_PREFIX=${CMAKE_INSTALL_PREFIX}
	BINARY_DIR ${PX4_BINARY_DIR}/build_gazebo
	INSTALL_COMMAND ""
	)
set_target_properties(sitl_gazebo PROPERTIES EXCLUDE_FROM_ALL TRUE)

ExternalProject_Add_Step(sitl_gazebo forceconfigure
	DEPENDEES update
	DEPENDERS configure
	ALWAYS 1)

# create targets for each viewer/model/debugger combination
set(viewers none jmavsim gazebo replay)
set(debuggers none ide gdb lldb ddd valgrind callgrind)
set(models none iris iris_opt_flow standard_vtol plane solo tailsitter typhoon_h480 rover)
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
				add_dependencies(${_targ_name} sitl_gazebo)
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
