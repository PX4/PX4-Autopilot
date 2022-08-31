
px4_add_git_submodule(TARGET git_jsbsim_bridge PATH "${PX4_SOURCE_DIR}/Tools/simulation/jsbsim/jsbsim_bridge")

include(ExternalProject)
ExternalProject_Add(jsbsim_bridge
	SOURCE_DIR ${PX4_SOURCE_DIR}/Tools/simulation/jsbsim/jsbsim_bridge
	CMAKE_ARGS -DCMAKE_INSTALL_PREFIX=${CMAKE_INSTALL_PREFIX}
	BINARY_DIR ${PX4_BINARY_DIR}/build_jsbsim_bridge
	INSTALL_COMMAND ""
	DEPENDS git_jsbsim_bridge
	USES_TERMINAL_CONFIGURE true
	USES_TERMINAL_BUILD true
	EXCLUDE_FROM_ALL true
	BUILD_ALWAYS 1
)

# jsbsim: create targets for jsbsim
set(models
	rascal
	quadrotor_x
	hexarotor_x
	malolo
)

set(worlds
	none
	LSZH
)

# default jsbsim target
add_custom_target(jsbsim
	COMMAND ${PX4_SOURCE_DIR}/Tools/simulation/jsbsim/sitl_run.sh $<TARGET_FILE:px4> "rascal" "LSZH" ${PX4_SOURCE_DIR} ${PX4_BINARY_DIR}
	WORKING_DIRECTORY ${SITL_WORKING_DIR}
	USES_TERMINAL
	DEPENDS px4 jsbsim_bridge
)

foreach(model ${models})
	foreach(world ${worlds})
		if(world STREQUAL "none")
			add_custom_target(jsbsim_${model}
				COMMAND ${PX4_SOURCE_DIR}/Tools/simulation/jsbsim/sitl_run.sh $<TARGET_FILE:px4> ${model} "LSZH" ${PX4_SOURCE_DIR} ${PX4_BINARY_DIR}
				WORKING_DIRECTORY ${SITL_WORKING_DIR}
				USES_TERMINAL
				DEPENDS px4 jsbsim_bridge
			)
		else()
			add_custom_target(jsbsim_${model}__${world}
				COMMAND ${PX4_SOURCE_DIR}/Tools/simulation/jsbsim/sitl_run.sh $<TARGET_FILE:px4> ${model} ${world} ${PX4_SOURCE_DIR} ${PX4_BINARY_DIR}
				WORKING_DIRECTORY ${SITL_WORKING_DIR}
				USES_TERMINAL
				DEPENDS px4 jsbsim_bridge
			)
		endif()
	endforeach()
endforeach()
