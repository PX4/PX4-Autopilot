macro(add_xplane_target TARGET_NAME SYS_AUTOSTART AIRFRAME_FILE)
	add_custom_target(${TARGET_NAME}
		COMMAND ${CMAKE_COMMAND} -E env PX4_SIMULATOR=xplane PX4_SYS_AUTOSTART=${SYS_AUTOSTART} $<TARGET_FILE:px4>
		WORKING_DIRECTORY ${SITL_WORKING_DIR}
		USES_TERMINAL
		DEPENDS
			px4
			${PX4_SOURCE_DIR}/ROMFS/px4fmu_common/init.d-posix/airframes/${AIRFRAME_FILE}
		COMMENT "Launching PX4 with X-Plane airframe (SYS_AUTOSTART=${SYS_AUTOSTART})"
	)
endmacro()

add_xplane_target(xplane_cessna172 5001 5001_xplane_cessna172)
add_xplane_target(xplane_tb2 5002 5002_xplane_tb2)
add_xplane_target(xplane_ehang184 5010 5010_xplane_ehang184)
add_xplane_target(xplane_alia250 5020 5020_xplane_alia250)
add_xplane_target(xplane_qtailsitter 5021 5021_xplane_qtailsitter)
