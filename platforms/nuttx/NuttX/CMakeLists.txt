cmake_minimum_required(VERSION 3.2)

if(NOT BOARD)
	message(FATAL_ERROR "BOARD must be set (eg px4fmu-v2)")
endif()

if(NOT nuttx_config_type)
	# default to nsh if not specified
	set(nuttx_config_type "nsh")
endif()

project(NuttX_${BOARD} LANGUAGES ASM C CXX)
message(STATUS "NuttX: " ${BOARD} " " ${nuttx_config_type} " " ${CMAKE_SYSTEM_PROCESSOR})

if (CMAKE_HOST_APPLE OR TRUE)
	# copy with rsync and create file dependencies
	set(cp_cmd "rsync")
	set(cp_opts)
	list(APPEND cp_opts
		-rp
		--inplace
	)
else()
	# copy with hard links
	# archive, recursive, force, link (hardlinks)
	set(cp_cmd "cp")
	set(cp_opts "-aRfl")
endif()

file(GLOB_RECURSE copy_nuttx_files
	LIST_DIRECTORIES false
	${CMAKE_CURRENT_SOURCE_DIR}/nuttx/*)

file(GLOB_RECURSE copy_apps_files
	LIST_DIRECTORIES false
	${CMAKE_CURRENT_SOURCE_DIR}/apps/*)

# copy nuttx to build directory
add_custom_command(OUTPUT nuttx_copy.stamp
	COMMAND ${cp_cmd} ${cp_opts} ${CMAKE_CURRENT_SOURCE_DIR}/nuttx ${CMAKE_CURRENT_BINARY_DIR}
	COMMAND cmake -E touch nuttx_copy.stamp
	DEPENDS ${copy_nuttx_files}
	COMMENT "Copying NuttX/nuttx to ${CMAKE_CURRENT_BINARY_DIR}"
	)
set(NUTTX_DIR ${CMAKE_CURRENT_BINARY_DIR}/nuttx)

# copy apps to build directory
add_custom_command(OUTPUT apps_copy.stamp
	COMMAND ${cp_cmd} ${cp_opts} ${CMAKE_CURRENT_SOURCE_DIR}/apps ${CMAKE_CURRENT_BINARY_DIR}
	COMMAND cmake -E touch apps_copy.stamp
	DEPENDS ${copy_apps_files}
	COMMENT "Copying NuttX/apps to ${CMAKE_CURRENT_BINARY_DIR}"
	)
set(APPS_DIR ${CMAKE_CURRENT_BINARY_DIR}/apps)

# copy PX4 board config into nuttx
file(GLOB_RECURSE board_config_files ${PX4_SOURCE_DIR}/nuttx-configs/${BOARD})
add_custom_command(
	OUTPUT
		${NUTTX_DIR}/PX4_Config.mk
		${NUTTX_DIR}/PX4_Warnings.mk
		${NUTTX_DIR}/configs/${BOARD}/${nuttx_config_type}/defconfig
		${NUTTX_DIR}/.config
	COMMAND ${CMAKE_COMMAND} -E create_symlink ${PX4_SOURCE_DIR}/nuttx-configs/PX4_Config.mk ${NUTTX_DIR}/PX4_Config.mk
	COMMAND ${CMAKE_COMMAND} -E create_symlink ${PX4_SOURCE_DIR}/nuttx-configs/PX4_Warnings.mk ${NUTTX_DIR}/PX4_Warnings.mk
	COMMAND ${cp_cmd} ${cp_opts} ${PX4_SOURCE_DIR}/nuttx-configs/${BOARD} ${NUTTX_DIR}/configs/
	COMMAND sh configure.sh ${BOARD}/${nuttx_config_type}
	DEPENDS
		${PX4_SOURCE_DIR}/nuttx-configs/PX4_Config.mk
		${PX4_SOURCE_DIR}/nuttx-configs/PX4_Warnings.mk
		${PX4_SOURCE_DIR}/nuttx-configs/${BOARD}/${nuttx_config_type}/defconfig
		${board_config_files}
		nuttx_copy.stamp apps_copy.stamp
	WORKING_DIRECTORY ${NUTTX_DIR}/tools
	COMMENT "Copying NuttX config ${BOARD} and configuring"
	)
add_custom_target(nuttx_configure DEPENDS ${NUTTX_DIR}/.config)

# context
add_custom_command(OUTPUT ${NUTTX_DIR}/include/nuttx/version.h ${NUTTX_DIR}/include/nuttx/config.h
	COMMAND make --no-print-directory --silent context > /dev/null
	DEPENDS nuttx_configure ${NUTTX_DIR}/.config
	WORKING_DIRECTORY ${NUTTX_DIR}
	)
add_custom_target(nuttx_context DEPENDS ${NUTTX_DIR}/include/nuttx/version.h)

# APPS

# add additional commands to nuttx builtins
set(builtin_registry ${APPS_DIR}/builtin/registry)
set(nuttx_builtin_list)
foreach(module ${module_libraries})
	get_target_property(MAIN ${module} MAIN)
	get_target_property(STACK_MAIN ${module} STACK_MAIN)
	get_target_property(PRIORITY ${module} PRIORITY)

	if(MAIN)
		add_custom_command(OUTPUT ${builtin_registry}/${MAIN}_main.bdat
			COMMAND echo "{ \"${MAIN}\", ${PRIORITY}, ${STACK_MAIN}, ${MAIN}_main }," > ${builtin_registry}/${MAIN}_main.bdat
			COMMAND cmake -E touch ${builtin_registry}/.updated
			VERBATIM
			)
		list(APPEND nuttx_builtin_list ${builtin_registry}/${MAIN}_main.bdat)

		add_custom_command(OUTPUT ${builtin_registry}/${MAIN}_main.pdat
			COMMAND echo "int ${MAIN}_main(int argc, char *argv[]);" > ${builtin_registry}/${MAIN}_main.pdat
			COMMAND cmake -E touch ${builtin_registry}/.updated
			VERBATIM
			)
		list(APPEND nuttx_builtin_list ${builtin_registry}/${MAIN}_main.pdat)
	endif()
endforeach()

# libapps.a
add_custom_command(OUTPUT ${APPS_DIR}/libapps.a
		${APPS_DIR}/nshlib/.built
		${APPS_DIR}/builtin/.built
		${APPS_DIR}/builtin/builtin_list.h
		${APPS_DIR}/builtin/builtin_proto.h
	COMMAND make --silent --no-print-directory -C ../apps TOPDIR="${NUTTX_DIR}" libapps.a > /dev/null
	DEPENDS nuttx_context ${nuttx_builtin_list}
	WORKING_DIRECTORY ${NUTTX_DIR}
	)
add_custom_target(nuttx_apps_build DEPENDS ${APPS_DIR}/libapps.a)
add_library(nuttx_apps STATIC IMPORTED GLOBAL)
set_property(TARGET nuttx_apps PROPERTY IMPORTED_LOCATION ${APPS_DIR}/libapps.a)

# libboard.a
add_custom_command(OUTPUT ${NUTTX_DIR}/arch/arm/src/board/libboard.a
	COMMAND make --silent --no-print-directory -C board TOPDIR="${NUTTX_DIR}" libboard.a EXTRADEFINES=-D__KERNEL__ > /dev/null
	DEPENDS nuttx_context
	WORKING_DIRECTORY ${NUTTX_DIR}/arch/arm/src
	)
add_custom_target(nuttx_board_build DEPENDS ${NUTTX_DIR}/arch/arm/src/board/libboard.a)
add_library(nuttx_board STATIC IMPORTED GLOBAL)
set_property(TARGET nuttx_board PROPERTY IMPORTED_LOCATION ${NUTTX_DIR}/arch/arm/src/board/libboard.a)

# helper for all targets
add_custom_target(nuttx_build DEPENDS nuttx_board_build nuttx_apps_build)
function(add_nuttx_dir nuttx_lib nuttx_lib_dir kernel extra)
	file(GLOB_RECURSE nuttx_lib_files
		LIST_DIRECTORIES false
		${CMAKE_CURRENT_SOURCE_DIR}/nuttx/${nuttx_lib_dir}/*)

	add_custom_command(OUTPUT ${NUTTX_DIR}/${nuttx_lib_dir}/lib${nuttx_lib}.a
		COMMAND find ${nuttx_lib_dir} -type f -name *.o -o -name .built -delete
		COMMAND make -C ${nuttx_lib_dir} -j2 --silent --no-print-directory lib${nuttx_lib}.a TOPDIR="${NUTTX_DIR}" KERNEL=${kernel} EXTRADEFINES=${extra} > /dev/null
		DEPENDS ${nuttx_lib_files} nuttx_context
		WORKING_DIRECTORY ${NUTTX_DIR}
		)
	add_custom_target(nuttx_${nuttx_lib}_build DEPENDS ${NUTTX_DIR}/${nuttx_lib_dir}/lib${nuttx_lib}.a)
	add_library(nuttx_${nuttx_lib} STATIC IMPORTED GLOBAL)
	set_property(TARGET nuttx_${nuttx_lib} PROPERTY IMPORTED_LOCATION ${NUTTX_DIR}/${nuttx_lib_dir}/lib${nuttx_lib}.a)
	add_dependencies(nuttx_build nuttx_${nuttx_lib}_build)
endfunction()

# add_nuttx_dir(NAME DIRECTORY KERNEL EXTRA)

add_nuttx_dir(arch arch/arm/src y -D__KERNEL__)
add_nuttx_dir(binfmt binfmt y -D__KERNEL__)
add_nuttx_dir(configs configs y -D__KERNEL__)
add_nuttx_dir(drivers drivers y -D__KERNEL__)
add_nuttx_dir(fs fs y -D__KERNEL__)
add_nuttx_dir(sched sched y -D__KERNEL__)

add_nuttx_dir(c libc n "")
add_nuttx_dir(cxx libxx n "")
add_nuttx_dir(mm mm n "")

# oldconfig helper
add_custom_target(oldconfig
	COMMAND make --no-print-directory -C ${NUTTX_DIR} CONFIG_ARCH_BOARD=${BOARD} oldconfig
	COMMAND cp ${NUTTX_DIR}/.config ${PX4_SOURCE_DIR}/nuttx-configs/${BOARD}/${nuttx_config_type}/defconfig
	COMMAND ${PX4_SOURCE_DIR}/Tools/nuttx_defconf_tool.sh ${PX4_SOURCE_DIR}/nuttx-configs/${BOARD}/${nuttx_config_type}/defconfig
	DEPENDS nuttx_configure
	WORKING_DIRECTORY ${NUTTX_DIR}
	COMMENT "Running NuttX make oldconfig for ${BOARD} with ${nuttx_config_type}"
	USES_TERMINAL
	)

# menuconfig helper
add_custom_target(menuconfig
	COMMAND make --no-print-directory -C ${NUTTX_DIR} CONFIG_ARCH_BOARD=${BOARD} menuconfig
	COMMAND cp ${NUTTX_DIR}/nuttx/.config ${PX4_SOURCE_DIR}/nuttx-configs/${BOARD}/${nuttx_config_type}/defconfig
	COMMAND ${PX4_SOURCE_DIR}/Tools/nuttx_defconf_tool.sh ${PX4_SOURCE_DIR}/nuttx-configs/${BOARD}/${nuttx_config_type}/defconfig
	DEPENDS nuttx_configure
	WORKING_DIRECTORY ${NUTTX_DIR}
	COMMENT "Running NuttX make menuconfig for ${BOARD} with ${nuttx_config_type}"
	USES_TERMINAL
	)