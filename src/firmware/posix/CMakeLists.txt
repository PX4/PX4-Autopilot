include (common/px4_upload)

list(APPEND CMAKE_MODULE_PATH ${CMAKE_CURRENT_SOURCE_DIR})

include_directories(${CMAKE_CURRENT_BINARY_DIR})

get_property(module_libraries GLOBAL PROPERTY PX4_MODULE_LIBRARIES)

px4_posix_generate_builtin_commands(
	OUT apps
	MODULE_LIST ${module_libraries}
	)

if ((${BOARD} STREQUAL "eagle") OR (${BOARD} STREQUAL "excelsior"))
	include(fastrpc)
	include(linux_app)

	FASTRPC_STUB_GEN(../qurt/px4muorb.idl)

	list(APPEND c_flags
		-Wno-missing-prototypes
		-Wno-missing-declarations
	)
	add_compile_options("$<$<COMPILE_LANGUAGE:C>:${c_flags}>")

	LINUX_APP(
		APP_NAME px4
		IDL_NAME px4muorb
		APPS_DEST "/home/linaro"
		SOURCES main.cpp apps.cpp px4muorb_stub.c
		LINK_LIBS ${FASTRPC_ARM_LIBS}
		)

	# TODO: update cmake_hexagon and merge with the regular linux app
	target_link_libraries(px4 ${module_libraries} ${df_driver_libs})
	target_link_libraries(px4 lib__ecl) # temporary until we can update ecl cmake
	target_link_libraries(px4 drivers_board modules__uORB parameters px4_layer work_queue pthread)
else()
	add_executable(px4 main.cpp apps.cpp)

	target_link_libraries(px4 PRIVATE ${module_libraries} ${df_driver_libs})
	target_link_libraries(px4 PUBLIC lib__ecl) # temporary until we can update ecl cmake
	target_link_libraries(px4 PUBLIC drivers_board modules__uORB parameters px4_layer work_queue pthread)
endif()

if(UNIX AND NOT APPLE)
   target_link_libraries(px4 PUBLIC rt)
endif()

# upload helpers
if ("${BOARD}" STREQUAL "rpi")

	target_link_libraries(px4 PRIVATE
		-B${CXX_COMPILER_PATH}/arm-linux-gnueabihf/libc/usr/lib
		-L${CXX_COMPILER_PATH}/arm-linux-gnueabihf/libc/usr/lib
	)

	file(GLOB RPI_CONFIG_FILES ${PX4_SOURCE_DIR}/posix-configs/rpi/*.config)
	px4_add_scp_push(
		OUT upload
		OS ${OS}
		BOARD ${BOARD}
		FILES $<TARGET_FILE:px4> ${RPI_CONFIG_FILES} ${PX4_SOURCE_DIR}/ROMFS
		DEPENDS px4
		DEST /home/pi
		)

elseif ("${BOARD}" STREQUAL "bebop")

	px4_add_adb_push_to_bebop(
		OUT upload
		OS ${OS}
		BOARD ${BOARD}
		FILES $<TARGET_FILE:px4>.stripped
		DEPENDS px4
		DEST /usr/bin
		)

elseif (${BOARD} STREQUAL "sitl")

	include(sitl_target)
	if(BUILD_TESTING)
		include(sitl_tests)
	endif()


elseif (${BOARD} STREQUAL "eagle")

	px4_add_adb_push(
		OUT upload
		OS ${OS}
		BOARD ${BOARD}
		FILES $<TARGET_FILE:px4> ${PX4_SOURCE_DIR}/posix-configs/eagle/flight/mainapp.config
		DEPENDS px4
		DEST /home/linaro
		)

elseif (${BOARD} STREQUAL "excelsior")

	px4_add_adb_push(
		OUT upload
		OS ${OS}
		BOARD ${BOARD}
		FILES $<TARGET_FILE:px4> ${PX4_SOURCE_DIR}/posix-configs/eagle/flight/mainapp.config
		DEPENDS px4
		DEST /home/linaro
		)
endif()

#=============================================================================
# install
#

install(TARGETS px4 DESTINATION ${CMAKE_INSTALL_BINDIR})
install(DIRECTORY ${PROJECT_SOURCE_DIR}/ROMFS DESTINATION ${CMAKE_INSTALL_DATADIR}/${PROJECT_NAME})
install(DIRECTORY ${PROJECT_SOURCE_DIR}/posix-configs DESTINATION ${CMAKE_INSTALL_DATADIR}/${PROJECT_NAME})

add_custom_command(OUTPUT px4.stripped
			COMMAND ${CMAKE_STRIP} -R .comment -R .gnu.version -o $<TARGET_FILE:px4>.stripped $<TARGET_FILE:px4>
			DEPENDS px4
			)
add_custom_target(strip DEPENDS px4.stripped)
