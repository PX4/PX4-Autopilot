if(ENABLE_LOCKSTEP_SCHEDULER STREQUAL "no")

	find_program(FGFS_PATH "fgfs")

	if(FGFS_PATH)

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

		# find corresponding airframes
		file(GLOB flightgear_airframes
			RELATIVE ${PX4_SOURCE_DIR}/ROMFS/px4fmu_common/init.d-posix/airframes
			${PX4_SOURCE_DIR}/ROMFS/px4fmu_common/init.d-posix/airframes/*_flightgear_*
		)

		# remove any .post files
		foreach(flightgear_airframe IN LISTS flightgear_airframes)
			if(flightgear_airframe MATCHES ".post")
				list(REMOVE_ITEM flightgear_airframes ${flightgear_airframe})
			endif()
		endforeach()
		list(REMOVE_DUPLICATES flightgear_airframes)

		# default flightgear target
		add_custom_target(flightgear
			COMMAND ${PX4_SOURCE_DIR}/Tools/simulation/flightgear/sitl_run.sh $<TARGET_FILE:px4> "rascal" ${PX4_SOURCE_DIR} ${PX4_BINARY_DIR}
			WORKING_DIRECTORY ${SITL_WORKING_DIR}
			USES_TERMINAL
			DEPENDS px4 flightgear_bridge
		)

		foreach(model ${models})

			# match model to airframe
			set(airframe_model_only)
			set(airframe_sys_autostart)
			set(flightgear_airframe_found)
			foreach(flightgear_airframe IN LISTS flightgear_airframes)

				string(REGEX REPLACE ".*_flightgear_" "" airframe_model_only ${flightgear_airframe})
				string(REGEX REPLACE "_flightgear_.*" "" airframe_sys_autostart ${flightgear_airframe})

				if(model STREQUAL ${airframe_model_only})
					set(flightgear_airframe_found ${flightgear_airframe})
					break()
				endif()
			endforeach()

			if(flightgear_airframe_found)
				#message(STATUS "flightgear model: ${model} (${airframe_model_only}), airframe: ${flightgear_airframe_found}, SYS_AUTOSTART: ${airframe_sys_autostart}")
			else()
				message(WARNING "flightgear missing model: ${model} (${airframe_model_only}), airframe: ${flightgear_airframe_found}, SYS_AUTOSTART: ${airframe_sys_autostart}")
			endif()

			add_custom_target(flightgear_${model}
				COMMAND ${PX4_SOURCE_DIR}/Tools/simulation/flightgear/sitl_run.sh $<TARGET_FILE:px4> ${model} ${PX4_SOURCE_DIR} ${PX4_BINARY_DIR}
				WORKING_DIRECTORY ${SITL_WORKING_DIR}
				USES_TERMINAL
				DEPENDS px4 flightgear_bridge
			)
		endforeach()
	endif()
endif()
