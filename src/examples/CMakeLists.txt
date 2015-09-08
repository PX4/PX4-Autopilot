set(directories
	./px4_simple_app
	./fixedwing_control
	./rover_steering_control
	./px4_mavlink_debug
	./px4_daemon_app
	./flow_position_estimator
	./matlab_csv_serial
	./subscriber
	./hwtest
	./publisher
	)

foreach(directory ${directories})
	add_subdirectory(${directory} EXCLUDE_FROM_ALL)
endforeach()
