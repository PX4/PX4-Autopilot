


if(DEFINED ENV{JSBSIM_ROOT_DIR} )
	set(JSBSIM_ROOT_DIR "$ENV{JSBSIM_ROOT_DIR}" )
endif()

find_path(JSBSIM_INCLUDE_DIR
	NAMES
		FGFDMExec.h
	PATHS
		${JSBSIM_ROOT_DIR}/include/JSBSim
		/usr/include/JSBSim
		/usr/local/include/JSBSim
)

if(JSBSIM_INCLUDE_DIR)

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


	# find corresponding airframes
	file(GLOB jsbsim_airframes
		RELATIVE ${PX4_SOURCE_DIR}/ROMFS/px4fmu_common/init.d-posix/airframes
		${PX4_SOURCE_DIR}/ROMFS/px4fmu_common/init.d-posix/airframes/*_jsbsim_*
	)

	# remove any .post files
	foreach(jsbsim_airframe IN LISTS jsbsim_airframes)
		if(jsbsim_airframe MATCHES ".post")
			list(REMOVE_ITEM jsbsim_airframes ${jsbsim_airframe})
		endif()
	endforeach()
	list(REMOVE_DUPLICATES jsbsim_airframes)

	# default jsbsim target
	add_custom_target(jsbsim
		COMMAND ${PX4_SOURCE_DIR}/Tools/simulation/jsbsim/sitl_run.sh $<TARGET_FILE:px4> "rascal" "LSZH" ${PX4_SOURCE_DIR} ${PX4_BINARY_DIR}
		WORKING_DIRECTORY ${SITL_WORKING_DIR}
		USES_TERMINAL
		DEPENDS px4 jsbsim_bridge
	)

	foreach(model ${models})

		# match model to airframe
		set(airframe_model_only)
		set(airframe_sys_autostart)
		set(jsbsim_airframe_found)
		foreach(jsbsim_airframe IN LISTS jsbsim_airframes)

			string(REGEX REPLACE ".*_jsbsim_" "" airframe_model_only ${jsbsim_airframe})
			string(REGEX REPLACE "_jsbsim_.*" "" airframe_sys_autostart ${jsbsim_airframe})

			if(model STREQUAL ${airframe_model_only})
				set(jsbsim_airframe_found ${jsbsim_airframe})
				break()
			endif()
		endforeach()

		if(jsbsim_airframe_found)
			#message(STATUS "jsbsim model: ${model} (${airframe_model_only}), airframe: ${jsbsim_airframe_found}, SYS_AUTOSTART: ${airframe_sys_autostart}")
		else()
			message(WARNING "jsbsim missing model: ${model} (${airframe_model_only}), airframe: ${jsbsim_airframe_found}, SYS_AUTOSTART: ${airframe_sys_autostart}")
		endif()


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

endif()
