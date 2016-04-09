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
	add_executable(mainapp
		${CMAKE_BINARY_DIR}/src/firmware/qurt/px4muorb_skel.c
		${CMAKE_BINARY_DIR}/apps.h)

	target_link_libraries(mainapp
		-Wl,--start-group
		${module_libraries}
                ${target_libraries}
                df_driver_framework
                ${df_driver_libs}
		-Wl,--end-group
		)
else()
	message("module_libraries = ${module_libraries}")
	message("target_libraries = ${target_libraries}")
	message("df_driver_libs = ${df_driver_libs}")
	# Generate the DSP lib and stubs but not the apps side executable
	# The Apps side executable is generated via the posix_eagle_xxxx target
	QURT_LIB(APP_NAME mainapp
		IDL_NAME px4muorb
		SOURCES
		${CMAKE_BINARY_DIR}/apps.h
		LINK_LIBS
		${module_libraries}
		${target_libraries}
		df_driver_framework
		${df_driver_libs}
		m
		)

	px4_add_adb_push(OUT upload
			 OS ${OS}
			 BOARD ${BOARD}
			 FILES ${CMAKE_CURRENT_BINARY_DIR}/libmainapp.so
				 ${CMAKE_CURRENT_BINARY_DIR}/libpx4muorb_skel.so
				 ${CMAKE_SOURCE_DIR}/posix-configs/eagle/flight/px4.config
			 DEPENDS build_mainapp_dsp build_mainapp_dsp
			 DEST /usr/share/data/adsp)
endif()

# vim: set noet ft=cmake fenc=utf-8 ff=unix :
