set(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} "${CMAKE_SOURCE_DIR}/cmake/cmake_hexagon")
include(fastrpc)
include(qurt_lib)

include_directories(${CMAKE_CURRENT_BINARY_DIR})

set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -Wno-missing-prototypes")

px4_qurt_generate_builtin_commands(
	OUT ${CMAKE_BINARY_DIR}/apps.h
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
		${CMAKE_BINARY_DIR}/src/firmware/qurt/px4muorb_skel.c
		${CMAKE_BINARY_DIR}/apps.h)

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
			${CMAKE_BINARY_DIR}/apps.h
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
				 ${CMAKE_SOURCE_DIR}/posix-configs/eagle/flight/px4.config
			 DEPENDS px4 px4muorb_skel
			 DEST /usr/share/data/adsp)
endif()

# vim: set noet ft=cmake fenc=utf-8 ff=unix :
