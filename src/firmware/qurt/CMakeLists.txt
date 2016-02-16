include_directories(${CMAKE_CURRENT_BINARY_DIR})

set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -Wno-missing-prototypes")

px4_qurt_generate_builtin_commands(
	OUT ${CMAKE_BINARY_DIR}/apps.h
	MODULE_LIST ${module_libraries})

# Enable build without HexagonSDK to check link dependencies
if ("${QURT_ENABLE_STUBS}" STREQUAL "1")
	add_definitions(-DQURT_EXE_BUILD)
	add_executable(mainapp 
		${CMAKE_SOURCE_DIR}/src/platforms/qurt/dspal/dspal_stub.c
		${CMAKE_BINARY_DIR}/apps.h)
else()
	QURT_BUNDLE(APP_NAME
		mainapp
		DSP_SOURCES
		${CMAKE_SOURCE_DIR}/src/platforms/qurt/dspal/dspal_stub.c
		${CMAKE_BINARY_DIR}/apps.h
		DSP_LINK_LIBS
		${module_libraries}
		${target_libraries}
		df_driver_framework
		m
		)
		
endif()

# vim: set noet ft=cmake fenc=utf-8 ff=unix :
