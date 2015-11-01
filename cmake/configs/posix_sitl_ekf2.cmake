include(cmake/configs/posix_sitl_simple.cmake)

list(APPEND config_module_list
	modules/attitude_estimator_ekf2
	)

set(config_sitl_rcS
	posix-configs/SITL/init/rcS_ekf2
	)
