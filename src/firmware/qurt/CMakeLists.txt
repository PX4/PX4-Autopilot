include_directories(${CMAKE_CURRENT_BINARY_DIR})

set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -Wno-missing-prototypes")

px4_qurt_generate_builtin_commands(
	OUT ${CMAKE_BINARY_DIR}/apps.h
	MODULE_LIST ${module_libraries})

FASTRPC_STUB_GEN(px4muorb.idl)

# Enable build without HexagonSDK to check link dependencies
if ("${QURT_ENABLE_STUBS}" STREQUAL "1")
	add_definitions(-DQURT_EXE_BUILD)
	add_executable(mainapp 
		${CMAKE_SOURCE_DIR}/src/platforms/qurt/dspal/dspal_stub.c
		${CMAKE_BINARY_DIR}/src/firmware/qurt/px4muorb_skel.c
		${CMAKE_BINARY_DIR}/apps.h)
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
		
endif()

# vim: set noet ft=cmake fenc=utf-8 ff=unix :
