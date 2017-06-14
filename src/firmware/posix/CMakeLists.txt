include (common/px4_upload)

include_directories(${CMAKE_CURRENT_BINARY_DIR})

px4_posix_generate_builtin_commands(
	OUT apps
	MODULE_LIST ${module_libraries})

# Define build target
set(APP_NAME px4)
set(MAIN_SRC ${PX4_SOURCE_DIR}/src/platforms/posix/main.cpp)
set(UPLOAD_NAME upload)

if ("${BOARD}" STREQUAL "eagle" OR ("${BOARD}" STREQUAL "excelsior"))

	include(fastrpc)
	include(linux_app)

	FASTRPC_STUB_GEN(../qurt/px4muorb.idl)

	set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -Wno-missing-prototypes -Wno-missing-declarations")
	LINUX_APP(
		APP_NAME ${APP_NAME}
		IDL_NAME px4muorb
		APPS_DEST "/home/linaro"
		SOURCES
			px4muorb_stub.c
			${MAIN_SRC}
			apps.cpp
		LINK_LIBS
			-Wl,--start-group
			${module_libraries}
			${df_driver_libs}
			${FASTRPC_ARM_LIBS}
			pthread m rt
			-Wl,--end-group
		)

	px4_add_adb_push(OUT ${UPLOAD_NAME}
				OS ${OS}
				BOARD ${BOARD}
				FILES ${CMAKE_CURRENT_BINARY_DIR}/${APP_NAME}
				${PX4_SOURCE_DIR}/posix-configs/eagle/flight/mainapp.config
				DEPENDS ${APP_NAME}
				DEST /home/linaro)

elseif ("${BOARD}" STREQUAL "rpi")

	px4_add_executable(${APP_NAME}
		${MAIN_SRC}
		apps.cpp
		)

	target_link_libraries(${APP_NAME}
		-Wl,--start-group
		${module_libraries}
		df_driver_framework
		${df_driver_libs}
		pthread m rt
		-Wl,--end-group
		)

	file(GLOB RPI_CONFIG_FILES ${PX4_SOURCE_DIR}/posix-configs/rpi/*.config)
	px4_add_scp_push(OUT ${UPLOAD_NAME}
				OS ${OS}
				BOARD ${BOARD}
				FILES ${CMAKE_CURRENT_BINARY_DIR}/${APP_NAME}
				${RPI_CONFIG_FILES}
				${PX4_SOURCE_DIR}/ROMFS
				DEPENDS ${APP_NAME}
				DEST /home/pi)

elseif ("${BOARD}" STREQUAL "bebop")

	set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -static")
	set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -static")

	px4_add_executable(${APP_NAME}
		${MAIN_SRC}
		apps.cpp
		)

	if (NOT APPLE)
		target_link_libraries(${APP_NAME}
			-Wl,--start-group
			${module_libraries}
			${df_driver_libs}
			pthread m rt
			-Wl,--end-group
			)
	else()
		target_link_libraries(${APP_NAME}
			${module_libraries}
			${df_driver_libs}
			pthread m
			)
	endif()

	px4_add_adb_push_to_bebop(OUT ${UPLOAD_NAME}
				OS ${OS}
				BOARD ${BOARD}
				FILES $<TARGET_FILE:px4>.stripped
				DEPENDS ${APP_NAME}.stripped
				DEST /usr/bin)

elseif ("${BOARD}" STREQUAL "sitl")

	include(./sitl_target.cmake)

else()

	px4_add_executable(${APP_NAME}
		${MAIN_SRC}
		apps.cpp
		)

	if (NOT APPLE)
		target_link_libraries(${APP_NAME}
			-Wl,--start-group
			${module_libraries}
			${df_driver_libs}
			pthread m rt
			-Wl,--end-group
			)
	else()
		target_link_libraries(${APP_NAME}
			${module_libraries}
			${df_driver_libs}
			pthread m
			)
	endif()
endif()

#=============================================================================
# install
#

install(TARGETS px4 DESTINATION ${CMAKE_INSTALL_BINDIR})
install(DIRECTORY ${PROJECT_SOURCE_DIR}/ROMFS DESTINATION ${CMAKE_INSTALL_DATADIR}/${PROJECT_NAME})
install(DIRECTORY ${PROJECT_SOURCE_DIR}/posix-configs DESTINATION ${CMAKE_INSTALL_DATADIR}/${PROJECT_NAME})

add_custom_target(strip
			DEPENDS "${APP_NAME}.stripped"
)

add_custom_command(OUTPUT "${APP_NAME}.stripped"
			COMMAND ${STRIP_TOOL} -R .comment -R .gnu.version -o $<TARGET_FILE:px4>.stripped $<TARGET_FILE:px4>
			DEPENDS px4)

# vim: set noet ft=cmake fenc=utf-8 ff=unix :
