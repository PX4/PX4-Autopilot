set(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} "${PX4_SOURCE_DIR}/cmake/cmake_hexagon")
include(common/px4_upload)
include(toolchain/Toolchain-qurt)
include(fastrpc)
include(qurt_lib)
include(qurt_flags)

include_directories(${CMAKE_CURRENT_BINARY_DIR})

set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -Wno-missing-prototypes")

px4_qurt_generate_builtin_commands(
	OUT ${PX4_BINARY_DIR}/apps
	MODULE_LIST ${module_libraries})

FASTRPC_STUB_GEN(px4muorb.idl)

# Enable build without HexagonSDK to check link dependencies
if ("${QURT_ENABLE_STUBS}" STREQUAL "1")
	add_definitions(-D QURT_EXE_BUILD=1)
	include_directories(
                ${CMAKE_CURRENT_BINARY_DIR}
                ${FASTRPC_DSP_INCLUDES}
                )
	add_executable(px4
		${PX4_BINARY_DIR}/src/firmware/qurt/px4muorb_skel.c
		${PX4_BINARY_DIR}/apps.cpp)

	target_link_libraries(px4
		-Wl,--start-group
		${module_libraries}
                ${target_libraries}
                ${df_driver_libs}
		-Wl,--end-group
		)
else()
	message("module_libraries = ${module_libraries}")
	message("target_libraries = ${target_libraries}")
	message("df_driver_libs = ${df_driver_libs}")
	message("module_external_libraries = ${module_external_libraries}")
	# Generate the DSP lib and stubs but not the apps side executable
	# The Apps side executable is generated via the posix_eagle_xxxx target
	QURT_LIB(LIB_NAME px4
		IDL_NAME px4muorb
		SOURCES
			${PX4_BINARY_DIR}/apps.cpp
		LINK_LIBS
			${module_libraries}
			${target_libraries}
			${df_driver_libs}
			m
			${module_external_libraries}
		)

	px4_add_adb_push(OUT upload
			 OS ${OS}
			 BOARD ${BOARD}
			 FILES ${CMAKE_CURRENT_BINARY_DIR}/libpx4.so
				 ${CMAKE_CURRENT_BINARY_DIR}/libpx4muorb_skel.so
				 ${PX4_SOURCE_DIR}/posix-configs/eagle/flight/px4.config
			 DEPENDS px4 px4muorb_skel
			 DEST /usr/share/data/adsp)
endif()

# vim: set noet ft=cmake fenc=utf-8 ff=unix :
