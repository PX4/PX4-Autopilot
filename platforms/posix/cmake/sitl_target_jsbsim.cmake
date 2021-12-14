
# Add support for external project building
include(ExternalProject)

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

set(debuggers
	none
	gdb
	lldb
	valgrind
	callgrind
)

# create targets for jsbsim
set(models_jsbsim
	rascal
	quadrotor_x
	hexarotor_x
	malolo
)

set(worlds_jsbsim
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
					DEPENDS
						jsbsim_bridge
						logs_symlink
						px4
				)

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
					DEPENDS
						jsbsim_bridge
						logs_symlink
						px4
				)
			endif()
		endforeach()
	endforeach()
endforeach()
