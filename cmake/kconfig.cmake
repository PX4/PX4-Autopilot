set(BOARD_DEFCONFIG ${PX4_CONFIG_FILE} CACHE FILEPATH "path to defconfig" FORCE)
set(BOARD_CONFIG ${PX4_BINARY_DIR}/boardconfig CACHE FILEPATH "path to config" FORCE)

find_program(MENUCONFIG_PATH menuconfig)
find_program(GUICONFIG_PATH guiconfig)
find_program(DEFCONFIG_PATH defconfig)
find_program(SAVEDEFCONFIG_PATH savedefconfig)
if(NOT MENUCONFIG_PATH OR NOT GUICONFIG_PATH OR NOT DEFCONFIG_PATH OR NOT SAVEDEFCONFIG_PATH)
    message(STATUS "kconfiglib is not installed\n"
                        "please install using \"pip3 install kconfiglib\"\n")
endif()


set(COMMON_KCONFIG_ENV_SETTINGS
	PYTHON_EXECUTABLE=${PYTHON_EXECUTABLE}
	KCONFIG_CONFIG=${BOARD_CONFIG}
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

if(EXISTS ${BOARD_DEFCONFIG})

    # Depend on BOARD_DEFCONFIG so that we reconfigure on config change
    set_property(DIRECTORY APPEND PROPERTY CMAKE_CONFIGURE_DEPENDS ${BOARD_DEFCONFIG})

    if(${LABEL} MATCHES "default" OR ${LABEL} MATCHES "bootloader" OR ${LABEL} MATCHES "canbootloader")
        # Generate boardconfig from saved defconfig
        execute_process(COMMAND ${CMAKE_COMMAND} -E env ${COMMON_KCONFIG_ENV_SETTINGS}
                        ${DEFCONFIG_PATH} ${BOARD_DEFCONFIG}
                        WORKING_DIRECTORY ${PX4_SOURCE_DIR}
                        OUTPUT_VARIABLE DUMMY_RESULTS)
    else()
        # Generate boardconfig from default.px4board and {label}.px4board
        execute_process(COMMAND ${CMAKE_COMMAND} -E env ${COMMON_KCONFIG_ENV_SETTINGS}
                        ${PYTHON_EXECUTABLE} ${PX4_SOURCE_DIR}/Tools/kconfig/merge_config.py Kconfig ${BOARD_CONFIG} ${PX4_BOARD_DIR}/default.px4board ${BOARD_DEFCONFIG}
                        WORKING_DIRECTORY ${PX4_SOURCE_DIR}
                        OUTPUT_VARIABLE DUMMY_RESULTS)
    endif()


    # parse board config options for cmake
    file(STRINGS ${BOARD_CONFIG} ConfigContents)
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
                set(${Name} ${Value} CACHE INTERNAL "BOARD DEFCONFIG: ${Name}" FORCE)
            endif()
        endif()

        # Find variable name
        string(REGEX MATCH "^CONFIG_BOARD_" Board ${NameAndValue})

        if(Board)
            string(REPLACE "CONFIG_BOARD_" "" ConfigKey ${Name})
            if(Value)
                set(${ConfigKey} ${Value})
                message(STATUS "${ConfigKey} ${Value}")
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
            string(REGEX REPLACE "(^[a-z]+)_([a-z0-9]+_[a-z0-9]+).*$" "\\1" driver_p1_folder ${driver})
            string(REGEX REPLACE "(^[a-z]+)_([a-z0-9]+_[a-z0-9]+).*$" "\\2" driver_p1_subfolder ${driver})

            # Pattern 2 XXX_XXX / XXXXXX
            string(REGEX REPLACE "(^[a-z]+_[a-z0-9]+)_([a-z0-9]+).*$" "\\1" driver_p2_folder ${driver})
            string(REGEX REPLACE "(^[a-z]+_[a-z0-9]+)_([a-z0-9]+).*$" "\\2" driver_p2_subfolder ${driver})

            # Pattern 3 XXXXXX / XXX_XXX / XXXXXX
            string(REGEX REPLACE "(^[a-z]+)_([a-z0-9]+_[a-z0-9]+)_([a-z]+[a-z0-9]+).*$" "\\1" driver_p3_folder ${driver})
            string(REGEX REPLACE "(^[a-z]+)_([a-z0-9]+_[a-z0-9]+)_([a-z]+[a-z0-9]+).*$" "\\2" driver_p3_subfolder ${driver})
            string(REGEX REPLACE "(^[a-z]+)_([a-z0-9]+_[a-z0-9]+)_([a-z]+[a-z0-9]+).*$" "\\3" driver_p3_subsubfolder ${driver})

            # Pattern 4 XXX_XXX / XXX_XXX_XXX
            string(REGEX REPLACE "(^[a-z]+_[a-z0-9]+)_([a-z_0-9]+).*$" "\\1" driver_p4_folder ${driver})
            string(REGEX REPLACE "(^[a-z]+_[a-z0-9]+)_([a-z_0-9]+).*$" "\\2" driver_p4_subfolder ${driver})

            # Pattern 5 XXXXXX / XXXXXX / XXX_XXX
            string(REGEX REPLACE "(^[a-z]+)_([a-z0-9]+[a-z0-9]+)_([a-z0-9]+_[a-z0-9]+).*$" "\\1" driver_p5_folder ${driver})
            string(REGEX REPLACE "(^[a-z]+)_([a-z0-9]+[a-z0-9]+)_([a-z0-9]+_[a-z0-9]+).*$" "\\2" driver_p5_subfolder ${driver})
            string(REGEX REPLACE "(^[a-z]+)_([a-z0-9]+[a-z0-9]+)_([a-z0-9]+_[a-z0-9]+).*$" "\\3" driver_p5_subsubfolder ${driver})

            # Trick circumvent PX4 src naming problem with underscores and slashes
            if(EXISTS ${PX4_SOURCE_DIR}/src/drivers/${driver})
                list(APPEND config_module_list drivers/${driver})
            elseif(EXISTS ${PX4_SOURCE_DIR}/src/drivers/${driver_path})
                list(APPEND config_module_list drivers/${driver_path})
            elseif(EXISTS ${PX4_SOURCE_DIR}/src/drivers/${driver_p3_folder}/${driver_p3_subfolder}/${driver_p3_subsubfolder})
                list(APPEND config_module_list drivers/${driver_p3_folder}/${driver_p3_subfolder}/${driver_p3_subsubfolder})
            elseif(EXISTS ${PX4_SOURCE_DIR}/src/drivers/${driver_p1_folder}/${driver_p1_subfolder})
                list(APPEND config_module_list drivers/${driver_p1_folder}/${driver_p1_subfolder})
            elseif(EXISTS ${PX4_SOURCE_DIR}/src/drivers/${driver_p4_folder}/${driver_p4_subfolder})
                list(APPEND config_module_list drivers/${driver_p4_folder}/${driver_p4_subfolder})
            elseif(EXISTS ${PX4_SOURCE_DIR}/src/drivers/${driver_p2_folder}/${driver_p2_subfolder})
                list(APPEND config_module_list drivers/${driver_p2_folder}/${driver_p2_subfolder})
            elseif(EXISTS ${PX4_SOURCE_DIR}/src/drivers/${driver_p5_folder}/${driver_p5_subfolder}/${driver_p5_subsubfolder})
                list(APPEND config_module_list drivers/${driver_p5_folder}/${driver_p5_subfolder}/${driver_p5_subsubfolder})
            else()
                message(FATAL_ERROR "Couldn't find path for ${driver}")
            endif()

        endif()

        # Find variable name
        string(REGEX MATCH "^CONFIG_MODULES[^=]+" Modules ${NameAndValue})

        if(Modules)
            # Find the value
            string(REPLACE "${Name}=" "" Value ${NameAndValue})
            string(REPLACE "CONFIG_MODULES_" "" module ${Name})
            string(TOLOWER ${module} module)

            list(APPEND config_module_list modules/${module})
        endif()

        # Find variable name
        string(REGEX MATCH "^CONFIG_SYSTEMCMDS[^=]+" Systemcmds ${NameAndValue})

        if(Systemcmds)
            # Find the value
            string(REPLACE "${Name}=" "" Value ${NameAndValue})
            string(REPLACE "CONFIG_SYSTEMCMDS_" "" systemcmd ${Name})
            string(TOLOWER ${systemcmd} systemcmd)

            list(APPEND config_module_list systemcmds/${systemcmd})
        endif()

        # Find variable name
        string(REGEX MATCH "^CONFIG_EXAMPLES[^=]+" Examples ${NameAndValue})

        if(Examples)
            # Find the value
            string(REPLACE "${Name}=" "" Value ${NameAndValue})
            string(REPLACE "CONFIG_EXAMPLES_" "" example ${Name})
            string(TOLOWER ${example} example)

            list(APPEND config_module_list examples/${example})
        endif()

    endforeach()

    if(PLATFORM)
        # set OS, and append specific platform module path
        set(PX4_PLATFORM ${PLATFORM} CACHE STRING "PX4 board OS" FORCE)
        list(APPEND CMAKE_MODULE_PATH ${PX4_SOURCE_DIR}/platforms/${PX4_PLATFORM}/cmake)

        # platform-specific include path
        include_directories(${PX4_SOURCE_DIR}/platforms/${PX4_PLATFORM}/src/px4/common/include)
    endif()

	if(ARCHITECTURE)
		set(CMAKE_SYSTEM_PROCESSOR ${ARCHITECTURE} CACHE INTERNAL "system processor" FORCE)
	endif()

	if(TOOLCHAIN)
		set(CMAKE_TOOLCHAIN_FILE Toolchain-${TOOLCHAIN} CACHE INTERNAL "toolchain file" FORCE)
	endif()

	set(romfs_extra_files)
	set(config_romfs_extra_dependencies)
	# additional embedded metadata
	if (NOT CONSTRAINED_FLASH AND NOT EXTERNAL_METADATA AND NOT ${PX4_BOARD_LABEL} STREQUAL "test")
		list(APPEND romfs_extra_files
			${PX4_BINARY_DIR}/parameters.json.xz
			${PX4_BINARY_DIR}/events/all_events.json.xz)
		list(APPEND romfs_extra_dependencies
			parameters_xml
			events_json)
	endif()
	list(APPEND romfs_extra_files ${PX4_BINARY_DIR}/component_general.json.xz)
	list(APPEND romfs_extra_dependencies component_general_json)
	set(config_romfs_extra_files ${romfs_extra_files} CACHE INTERNAL "extra ROMFS files" FORCE)
	set(config_romfs_extra_dependencies ${romfs_extra_dependencies} CACHE INTERNAL "extra ROMFS deps" FORCE)

	if(SERIAL_PORTS)
		set(board_serial_ports ${SERIAL_PORTS} PARENT_SCOPE)
	endif()

	# Serial ports
	set(board_serial_ports)
	if(SERIAL_GPS1)
        list(APPEND board_serial_ports GPS1:${SERIAL_GPS1})
	endif()
	if(SERIAL_GPS2)
        list(APPEND board_serial_ports GPS2:${SERIAL_GPS2})
	endif()
	if(SERIAL_GPS3)
        list(APPEND board_serial_ports GPS3:${SERIAL_GPS3})
	endif()
	if(SERIAL_GPS4)
        list(APPEND board_serial_ports GPS4:${SERIAL_GPS4})
	endif()
	if(SERIAL_GPS5)
        list(APPEND board_serial_ports GPS5:${SERIAL_GPS5})
	endif()
	if(SERIAL_TEL1)
        list(APPEND board_serial_ports TEL1:${SERIAL_TEL1})
	endif()
	if(SERIAL_TEL2)
        list(APPEND board_serial_ports TEL2:${SERIAL_TEL2})
	endif()
	if(SERIAL_TEL3)
        list(APPEND board_serial_ports TEL3:${SERIAL_TEL3})
	endif()
	if(SERIAL_TEL4)
        list(APPEND board_serial_ports TEL4:${SERIAL_TEL4})
	endif()
	if(SERIAL_TEL5)
        list(APPEND board_serial_ports TEL5:${SERIAL_TEL5})
	endif()


	# ROMFS
	if(ROMFSROOT)
		set(config_romfs_root ${ROMFSROOT} CACHE INTERNAL "ROMFS root" FORCE)

		if(BUILD_BOOTLOADER)
			set(config_build_bootloader "1" CACHE INTERNAL "build bootloader" FORCE)
		endif()

		# IO board (placed in ROMFS)
		if(IO)
			set(config_io_board ${IO} CACHE INTERNAL "IO" FORCE)
		endif()

		if(UAVCAN_PERIPHERALS)
			set(config_uavcan_peripheral_firmware ${UAVCAN_PERIPHERALS} CACHE INTERNAL "UAVCAN peripheral firmware" FORCE)
		endif()
	endif()

	if(UAVCAN_INTERFACES)
		set(config_uavcan_num_ifaces ${UAVCAN_INTERFACES} CACHE INTERNAL "UAVCAN interfaces" FORCE)
	endif()

	if(UAVCAN_TIMER_OVERRIDE)
		set(config_uavcan_timer_override ${UAVCAN_TIMER_OVERRIDE} CACHE INTERNAL "UAVCAN TIMER OVERRIDE" FORCE)
	endif()

	# OPTIONS

	if(CONSTRAINED_FLASH)
		set(px4_constrained_flash_build "1" CACHE INTERNAL "constrained flash build" FORCE)
		add_definitions(-DCONSTRAINED_FLASH)
		if (NO_HELP)
			add_definitions(-DCONSTRAINED_FLASH_NO_HELP="https://docs.px4.io/master/en/modules/modules_main.html")
		endif()
	endif()

	if(CONSTRAINED_MEMORY)
		set(px4_constrained_memory_build "1" CACHE INTERNAL "constrained memory build" FORCE)
		add_definitions(-DCONSTRAINED_MEMORY)
	endif()

	if(TESTING)
		set(PX4_TESTING "1" CACHE INTERNAL "testing enabled" FORCE)
	endif()

	if(ETHERNET)
		set(PX4_ETHERNET "1" CACHE INTERNAL "ethernet enabled" FORCE)
	endif()

	if(CRYPTO)
		set(PX4_CRYPTO ${CRYPTO} CACHE STRING "PX4 crypto implementation" FORCE)
	endif()

	if(KEYSTORE)
		set(PX4_KEYSTORE ${KEYSTORE} CACHE STRING "PX4 keystore implementation" FORCE)
	endif()

	if(LINKER_PREFIX)
		set(PX4_BOARD_LINKER_PREFIX ${LINKER_PREFIX} CACHE STRING "PX4 board linker prefix" FORCE)
	else()
		set(PX4_BOARD_LINKER_PREFIX "" CACHE STRING "PX4 board linker prefix" FORCE)
	endif()

	if(COMPILE_DEFINITIONS)
        add_definitions( ${COMPILE_DEFINITIONS})
	endif()

	if(LINUX)
        add_definitions( "-D__PX4_LINUX" )
	endif()

	if(LOCKSTEP)
        set(ENABLE_LOCKSTEP_SCHEDULER yes)
	endif()

	if(NOLOCKSTEP)
        set(ENABLE_LOCKSTEP_SCHEDULER no)
	endif()

	if(FULL_OPTIMIZATION)
        set(CMAKE_BUILD_TYPE "Release" CACHE STRING "Build type" FORCE)
	endif()

	include(px4_impl_os)
	px4_os_prebuild_targets(OUT prebuild_targets BOARD ${PX4_BOARD})

    # add board config directory src to build modules
	file(RELATIVE_PATH board_support_src_rel ${PX4_SOURCE_DIR}/src ${PX4_BOARD_DIR})
	list(APPEND config_module_list ${board_support_src_rel}/src)

	set(config_module_list ${config_module_list})

endif()


if(${LABEL} MATCHES "default" OR ${LABEL} MATCHES "bootloader" OR ${LABEL} MATCHES "canbootloader")
    add_custom_target(boardconfig
        ${CMAKE_COMMAND} -E env
        ${COMMON_KCONFIG_ENV_SETTINGS}
        ${MENUCONFIG_PATH} Kconfig
        COMMAND ${CMAKE_COMMAND} -E env ${COMMON_KCONFIG_ENV_SETTINGS} ${SAVEDEFCONFIG_PATH}
        COMMAND ${CMAKE_COMMAND} -E copy defconfig ${BOARD_DEFCONFIG}
        COMMAND ${CMAKE_COMMAND} -E rm ${PX4_BINARY_DIR}/NuttX/apps_copy.stamp
        WORKING_DIRECTORY ${PX4_SOURCE_DIR}
        USES_TERMINAL
        COMMAND_EXPAND_LISTS
    )

    add_custom_target(boardguiconfig
        ${CMAKE_COMMAND} -E env
        ${COMMON_KCONFIG_ENV_SETTINGS}
        ${GUICONFIG_PATH} Kconfig
        COMMAND ${CMAKE_COMMAND} -E env ${COMMON_KCONFIG_ENV_SETTINGS} ${SAVEDEFCONFIG_PATH}
        COMMAND ${CMAKE_COMMAND} -E copy defconfig ${BOARD_DEFCONFIG}
        COMMAND ${CMAKE_COMMAND} -E rm ${PX4_BINARY_DIR}/NuttX/apps_copy.stamp
        WORKING_DIRECTORY ${PX4_SOURCE_DIR}
        USES_TERMINAL
        COMMAND_EXPAND_LISTS
    )
else()
    add_custom_target(boardconfig
        ${CMAKE_COMMAND} -E env
        ${COMMON_KCONFIG_ENV_SETTINGS}
        ${MENUCONFIG_PATH} Kconfig
        COMMAND ${CMAKE_COMMAND} -E env ${COMMON_KCONFIG_ENV_SETTINGS} ${SAVEDEFCONFIG_PATH}
        COMMAND ${CMAKE_COMMAND} -E env ${COMMON_KCONFIG_ENV_SETTINGS} ${PYTHON_EXECUTABLE} ${PX4_SOURCE_DIR}/Tools/kconfig/diffconfig.py -m ${PX4_BOARD_DIR}/default.px4board defconfig > ${BOARD_DEFCONFIG}
        COMMAND ${CMAKE_COMMAND} -E rm ${PX4_BINARY_DIR}/NuttX/apps_copy.stamp
        WORKING_DIRECTORY ${PX4_SOURCE_DIR}
        USES_TERMINAL
        COMMAND_EXPAND_LISTS
    )

    add_custom_target(boardguiconfig
        ${CMAKE_COMMAND} -E env
        ${COMMON_KCONFIG_ENV_SETTINGS}
        ${GUICONFIG_PATH} Kconfig
        COMMAND ${CMAKE_COMMAND} -E env ${COMMON_KCONFIG_ENV_SETTINGS} ${SAVEDEFCONFIG_PATH}
        COMMAND ${CMAKE_COMMAND} -E env ${COMMON_KCONFIG_ENV_SETTINGS} ${PYTHON_EXECUTABLE} ${PX4_SOURCE_DIR}/Tools/kconfig/diffconfig.py -m ${PX4_BOARD_DIR}/default.px4board defconfig > ${BOARD_DEFCONFIG}
        COMMAND ${CMAKE_COMMAND} -E rm ${PX4_BINARY_DIR}/NuttX/apps_copy.stamp
        WORKING_DIRECTORY ${PX4_SOURCE_DIR}
        USES_TERMINAL
        COMMAND_EXPAND_LISTS
    )
endif()
