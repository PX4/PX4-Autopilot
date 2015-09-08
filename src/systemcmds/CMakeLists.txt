
set(directories
	./mtd
	./ver
	./mixer
	./tests
	./nshterm
	./config
	./top
	./bl_update
	./reboot
	./i2c
	./param
	./reflect
	./usb_connected
	./motor_test
	./esc_calib
	./perf
	./topic_listener
	./dumpfile
	./pwm
	)

foreach(directory ${directories})
	add_subdirectory(${directory} EXCLUDE_FROM_ALL)
endforeach()
