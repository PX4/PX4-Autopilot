

# add flighgear targets
if(ENABLE_LOCKSTEP_SCHEDULER STREQUAL "no")

	# Add support for external project building
	include(ExternalProject)

	px4_add_git_submodule(TARGET git_flightgear_bridge PATH "${PX4_SOURCE_DIR}/Tools/flightgear_bridge")

	ExternalProject_Add(flightgear_bridge
		SOURCE_DIR ${PX4_SOURCE_DIR}/Tools/flightgear_bridge
		CMAKE_ARGS -DCMAKE_INSTALL_PREFIX=${CMAKE_INSTALL_PREFIX}
		BINARY_DIR ${PX4_BINARY_DIR}/build_flightgear_bridge
		INSTALL_COMMAND ""
		DEPENDS git_flightgear_bridge
		USES_TERMINAL_CONFIGURE true
		USES_TERMINAL_BUILD true
		EXCLUDE_FROM_ALL true
		BUILD_ALWAYS 1
	)

	set(models
		rascal
		rascal-electric
		tf-g1
		tf-r1
	)

	foreach(model ${models})

		add_custom_target(flightgear_${model}
			COMMAND
				${PX4_SOURCE_DIR}/Tools/sitl_run.sh $<TARGET_FILE:px4> none flightgear ${model} none ${PX4_SOURCE_DIR} ${PX4_BINARY_DIR}
			WORKING_DIRECTORY ${SITL_WORKING_DIR}
			USES_TERMINAL
			DEPENDS
				flightgear_bridge
				logs_symlink
				px4
		)

	endforeach()
endif()
