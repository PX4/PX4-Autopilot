
add_subdirectory(src)

include_directories(${CMAKE_CURRENT_BINARY_DIR})

get_property(module_libraries GLOBAL PROPERTY PX4_LIBRARIES)

px4_posix_generate_builtin_commands(
	OUT apps
	MODULE_LIST ${module_libraries})

if (("${BOARD}" STREQUAL "eagle") OR ("${BOARD}" STREQUAL "excelsior"))
	include(fastrpc)
	include(linux_app)

	FASTRPC_STUB_GEN(../qurt/px4muorb.idl)

	set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -Wno-missing-prototypes -Wno-missing-declarations")
	LINUX_APP(
		APP_NAME px4
		IDL_NAME px4muorb
		APPS_DEST "/home/linaro"
		SOURCES
			px4muorb_stub.c
			src/main.cpp
			apps.cpp
		LINK_LIBS
			-Wl,--start-group
				${module_libraries}
				${df_driver_libs}
				${FASTRPC_ARM_LIBS}
				pthread m rt
			-Wl,--end-group
		)

	add_custom_target(upload
		COMMAND ${PX4_SOURCE_DIR}/Tools/adb_upload.sh $<TARGET_FILE:px4> ${PX4_SOURCE_DIR}/posix-configs/eagle/flight/mainapp.config /home/linaro
		DEPENDS px4
		COMMENT "uploading px4"
		USES_TERMINAL
		)

else()
	px4_add_executable(px4
		src/main.cpp
		apps.cpp
		)

	if (APPLE)
		target_link_libraries(px4
			${module_libraries}
			df_driver_framework
			${df_driver_libs}
			pthread m
		)
	else()
		target_link_libraries(px4
			-Wl,--start-group
				${module_libraries}
				df_driver_framework
				${df_driver_libs}
				pthread m rt
			-Wl,--end-group
		)
	endif()
endif()

if ("${BOARD}" STREQUAL "rpi")
	add_custom_target(upload
		COMMAND scp -r $<TARGET_FILE:px4> ${PX4_SOURCE_DIR}/posix-configs/rpi/*.config ${PX4_SOURCE_DIR}/ROMFS pi@navio:/home/pi
		DEPENDS px4
		COMMENT "uploading px4"
		USES_TERMINAL
		)

elseif ("${BOARD}" STREQUAL "bebop")

	add_custom_target(upload
		COMMAND ${CMAKE_STRIP} -R .comment -R .gnu.version -o $<TARGET_FILE:px4>.stripped $<TARGET_FILE:px4>
		COMMAND ${PX4_SOURCE_DIR}/Tools/adb_upload_to_bebop.sh $<TARGET_FILE:px4>.stripped /usr/bin
		DEPENDS px4
		COMMENT "uploading px4"
		USES_TERMINAL
		)

elseif ("${BOARD}" STREQUAL "sitl")

	include(sitl_target)
	if(BUILD_TESTING)
		include(sitl_tests)
	endif()

endif()

#=============================================================================
# install
#

install(TARGETS px4 DESTINATION ${CMAKE_INSTALL_BINDIR})
install(DIRECTORY ${PROJECT_SOURCE_DIR}/ROMFS DESTINATION ${CMAKE_INSTALL_DATADIR}/${PROJECT_NAME})
install(DIRECTORY ${PROJECT_SOURCE_DIR}/posix-configs DESTINATION ${CMAKE_INSTALL_DATADIR}/${PROJECT_NAME})


