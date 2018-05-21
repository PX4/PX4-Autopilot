set(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} "${PX4_SOURCE_DIR}/cmake/cmake_hexagon")
include(toolchain/Toolchain-qurt)
include(fastrpc)
include(qurt_lib)
include(qurt_flags)

add_subdirectory(src)

include_directories(${CMAKE_CURRENT_BINARY_DIR})

get_property(module_libraries GLOBAL PROPERTY PX4_MODULE_LIBRARIES)

px4_qurt_generate_builtin_commands(
	OUT ${PX4_BINARY_DIR}/apps
	MODULE_LIST ${module_libraries}
)

# Build px4 shared library that is run on the DSP
add_library(px4 SHARED ${PX4_BINARY_DIR}/apps.cpp px4muorb_stub.c)
target_link_libraries(px4 PRIVATE ${module_libraries} ${df_driver_libs})
target_include_directories(px4 PRIVATE ${FASTRPC_DSP_INCLUDES})
target_compile_options(px4 PRIVATE -Wno-missing-prototypes)

# px4muorb_skel.so
include(fastrpc)
FASTRPC_STUB_GEN(${PX4_SOURCE_DIR}/src/modules/muorb/px4muorb.idl)

add_library(px4muorb_skel SHARED px4muorb_skel.c)
target_include_directories(px4muorb_skel PRIVATE ${FASTRPC_DSP_INCLUDES})
target_compile_options(px4muorb_skel PRIVATE -Wno-missing-prototypes)
add_dependencies(px4muorb_skel generate_px4muorb_stubs)
target_link_libraries(px4muorb_skel ${CMAKE_CURRENT_BINARY_DIR}/libpx4.so)

# upload helper
set(DSPLIB_TARGET_PATH "/usr/share/data/adsp/")
add_custom_target(upload
	COMMAND adb wait-for-device
	COMMAND adb push $<TARGET_FILE:px4> ${DSPLIB_TARGET_PATH}
	COMMAND adb push $<TARGET_FILE:px4muorb_skel> ${DSPLIB_TARGET_PATH}
	COMMAND adb push ${PX4_SOURCE_DIR}/posix-configs/eagle/flight/px4.config ${DSPLIB_TARGET_PATH}
	COMMAND adb shell sync
	DEPENDS px4 px4muorb_skel
	COMMENT "uploading px4"
	USES_TERMINAL
)
