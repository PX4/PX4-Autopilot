add_custom_target(upload
        COMMAND rsync -arh --progress ${CMAKE_RUNTIME_OUTPUT_DIRECTORY} ${PX4_SOURCE_DIR}/posix-configs/rpi/*.config ${PX4_SOURCE_DIR}/ROMFS pi@"$ENV{AUTOPILOT_HOST}":/home/pi/px4
	DEPENDS px4
	COMMENT "uploading px4"
	USES_TERMINAL
	)
