include_directories(${CMAKE_CURRENT_BINARY_DIR})

set(V_ARCH v5)
set(HEXAGON_TOOLS_ROOT /opt/6.4.03)
set(TOOLSLIB 
	${HEXAGON_TOOLS_ROOT}/dinkumware/lib/${V_ARCH}/G0)

px4_qurt_generate_builtin_commands(
	OUT ${CMAKE_BINARY_DIR}/apps.h
	MODULE_LIST ${module_libraries})

# Clear -rdynamic flag which fails for hexagon
# this change is scoped in this directory and below
set(CMAKE_SHARED_LIBRARY_LINK_C_FLAGS "")
set(CMAKE_SHARED_LIBRARY_LINK_CXX_FLAGS "")

px4_generate_parameters()

# Enable build without HexagonSDK to check link dependencies
if ("${QURT_ENABLE_STUBS}" STREQUAL "1")
	add_executable(mainapp 
		${CMAKE_SOURCE_DIR}/src/platforms/qurt/dspal/dspal_stub.c
		${CMAKE_BINARY_DIR}/px4_parameters.c
		${CMAKE_BINARY_DIR}/apps.h)
else("${QURT_ENABLE_STUBS}" STREQUAL "1")
	add_library(mainapp 
		${CMAKE_SOURCE_DIR}/src/platforms/qurt/dspal/dspal_stub.c
		${CMAKE_BINARY_DIR}/px4_parameters.c
		${CMAKE_BINARY_DIR}/apps.h)
endif()

target_link_libraries(mainapp
	-Wl,--whole-archive
	${module_libraries}
	${target_libraries}
	m
	-Wl,--no-whole-archive
	-Wl,${TOOLSLIB}/pic/libstdc++.a)

# vim: set noet ft=cmake fenc=utf-8 ff=unix :
