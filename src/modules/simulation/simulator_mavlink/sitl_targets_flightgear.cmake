if(ENABLE_LOCKSTEP_SCHEDULER STREQUAL "no")

	px4_add_git_submodule(TARGET git_flightgear_bridge PATH "${PX4_SOURCE_DIR}/Tools/simulation/flightgear/flightgear_bridge")

	include(ExternalProject)
	ExternalProject_Add(flightgear_bridge
		SOURCE_DIR ${PX4_SOURCE_DIR}/Tools/simulation/flightgear/flightgear_bridge
		CMAKE_ARGS -DCMAKE_INSTALL_PREFIX=${CMAKE_INSTALL_PREFIX}
		BINARY_DIR ${PX4_BINARY_DIR}/build_flightgear_bridge
		INSTALL_COMMAND ""
		DEPENDS git_flightgear_bridge
		USES_TERMINAL_CONFIGURE true
		USES_TERMINAL_BUILD true
		EXCLUDE_FROM_ALL true
		BUILD_ALWAYS 1
	)

	# flighgear targets
	set(models
		rascal
		rascal-electric
		tf-g1
		tf-g2
		tf-r1
	)

	# default flightgear target
	add_custom_target(flightgear
		COMMAND ${PX4_SOURCE_DIR}/Tools/simulation/flightgear/sitl_run.sh $<TARGET_FILE:px4> "rascal" ${PX4_SOURCE_DIR} ${PX4_BINARY_DIR}
		WORKING_DIRECTORY ${SITL_WORKING_DIR}
		USES_TERMINAL
		DEPENDS px4 flightgear_bridge
	)

	foreach(model ${models})
		add_custom_target(flightgear_${model}
			COMMAND ${PX4_SOURCE_DIR}/Tools/simulation/flightgear/sitl_run.sh $<TARGET_FILE:px4> ${model} ${PX4_SOURCE_DIR} ${PX4_BINARY_DIR}
			WORKING_DIRECTORY ${SITL_WORKING_DIR}
			USES_TERMINAL
			DEPENDS px4 flightgear_bridge
		)
	endforeach()
endif()
