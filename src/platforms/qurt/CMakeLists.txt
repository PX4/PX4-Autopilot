set(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} "${PX4_SOURCE_DIR}/cmake/cmake_hexagon")
include(toolchain/Toolchain-qurt)
include(fastrpc)
include(qurt_lib)
include(qurt_flags)

add_subdirectory(src)

include_directories(${CMAKE_CURRENT_BINARY_DIR})

set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -Wno-missing-prototypes")

get_property(module_libraries GLOBAL PROPERTY PX4_MODULE_LIBRARIES)

px4_qurt_generate_builtin_commands(
	OUT ${PX4_BINARY_DIR}/apps
	MODULE_LIST ${module_libraries}
)

FASTRPC_STUB_GEN(px4muorb.idl)

# Enable build without HexagonSDK to check link dependencies
if ("${QURT_ENABLE_STUBS}" STREQUAL "1")

	add_definitions(-D QURT_EXE_BUILD=1)

	include_directories(
                ${CMAKE_CURRENT_BINARY_DIR}
                ${FASTRPC_DSP_INCLUDES}
                )

	add_executable(px4
		${PX4_BINARY_DIR}/apps.cpp
		${PX4_BINARY_DIR}/platforms/qurt/px4muorb_skel.c
	)

	target_link_libraries(px4 PRIVATE ${module_libraries} ${df_driver_libs})
else()
	# Generate the DSP lib and stubs but not the apps side executable
	# The Apps side executable is generated via the posix_eagle_xxxx target
	QURT_LIB(LIB_NAME px4
		IDL_NAME px4muorb
		SOURCES
			${PX4_BINARY_DIR}/apps.cpp
		LINK_LIBS
			${module_libraries}
			${df_driver_libs}
			m
			modules__muorb__adsp
		)

	add_custom_target(upload
		COMMAND
			${PX4_SOURCE_DIR}/Tools/adb_upload.sh
				${CMAKE_CURRENT_BINARY_DIR}/libpx4.so ${CMAKE_CURRENT_BINARY_DIR}/libpx4muorb_skel.so ${PX4_SOURCE_DIR}/posix-configs/eagle/flight/px4.config
				/usr/share/data/adsp
		DEPENDS px4 px4muorb_skel
		COMMENT "uploading px4"
		USES_TERMINAL
	)

endif()
