set(BOARD_DEFCONFIG ${PX4_BOARD_DIR}/${PX4_BOARD_LABEL}-boardconfig CACHE FILEPATH "path to defconfig" FORCE)

set(COMMON_KCONFIG_ENV_SETTINGS
	PYTHON_EXECUTABLE=${PYTHON_EXECUTABLE}
	KCONFIG_CONFIG=${BOARD_DEFCONFIG}
	# Set environment variables so that Kconfig can prune Kconfig source
	# files for other architectures
	PLATFORM=${PX4_PLATFORM}
	VENDOR=${PX4_BOARD_VENDOR}
	MODEL=${PX4_BOARD_MODEL}
	LABEL=${PX4_BOARD_LABEL}
	TOOLCHAIN=${CMAKE_TOOLCHAIN_FILE}
	ARCHITECTURE=${CMAKE_SYSTEM_PROCESSOR}
	ROMFSROOT=${config_romfs_root}
)

add_custom_target(boardconfig
	${CMAKE_COMMAND} -E env
	${COMMON_KCONFIG_ENV_SETTINGS}
	${PYTHON_EXECUTABLE} ${PX4_SOURCE_DIR}/platforms/nuttx/NuttX/tools/menuconfig.py Kconfig
	WORKING_DIRECTORY ${PX4_SOURCE_DIR}
	USES_TERMINAL
	COMMAND_EXPAND_LISTS
)

# parse board config options for cmake
file(STRINGS ${BOARD_DEFCONFIG} ConfigContents)
foreach(NameAndValue ${ConfigContents})
	# Strip leading spaces
	string(REGEX REPLACE "^[ ]+" "" NameAndValue ${NameAndValue})

	# Find variable name
	string(REGEX MATCH "^CONFIG[^=]+" Name ${NameAndValue})

	if(Name)
		# Find the value
		string(REPLACE "${Name}=" "" Value ${NameAndValue})

		if(Value)
			# remove extra quotes
			string(REPLACE "\"" "" Value ${Value})

			# Set the variable
			#message(STATUS "${Name} ${Value}")
			set(${Name} ${Value} CACHE INTERNAL "BOARD DEFCONFIG: ${Name}" FORCE)
		endif()
	endif()

	# Find variable name
	string(REGEX MATCH "^CONFIG_DRIVERS[^=]+" Drivers ${NameAndValue})

	if(Drivers)
		# Find the value
		string(REPLACE "${Name}=" "" Value ${NameAndValue})
		string(REPLACE "CONFIG_DRIVERS_" "" driver ${Name})
		string(TOLOWER ${driver} driver)
		
		string(REPLACE "_" "/" driver_path ${driver})
		
		# Pattern 1 XXX / XXX_XXX
		string(REGEX REPLACE "(^[a-z]+)_([a-z]+_[a-z0-9]+).*$" "\\1" driver_p1_folder ${driver})
		string(REGEX REPLACE "(^[a-z]+)_([a-z]+_[a-z0-9]+).*$" "\\2" driver_p1_subfolder ${driver})
		
		# Pattern 2 XXX_XXX / XXXXXX
		string(REGEX REPLACE "(^[a-z]+_[a-z]+)_([a-z0-9]+).*$" "\\1" driver_p2_folder ${driver})
		string(REGEX REPLACE "(^[a-z]+_[a-z]+)_([a-z0-9]+).*$" "\\2" driver_p2_subfolder ${driver})

		# Trick circumvent PX4 src naming problem with underscores and slashes
		if(EXISTS ${PX4_SOURCE_DIR}/src/drivers/${driver})
			list(APPEND config_module_list drivers/${driver})
		elseif(EXISTS ${PX4_SOURCE_DIR}/src/drivers/${driver_path})
			list(APPEND config_module_list drivers/${driver_path})
		elseif(EXISTS ${PX4_SOURCE_DIR}/src/drivers/${driver_p1_folder}/${driver_p1_subfolder})
			list(APPEND config_module_list drivers/${driver_p1_folder}/${driver_p1_subfolder})
		elseif(EXISTS ${PX4_SOURCE_DIR}/src/drivers/${driver_p2_folder}/${driver_p2_subfolder})
			list(APPEND config_module_list drivers/${driver_p2_folder}/${driver_p2_subfolder})
		else()
			message(FATAL_ERROR "Couldn't find path for ${driver}")
		endif()
		
	endif()
endforeach()
