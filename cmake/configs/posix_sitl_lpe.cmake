include(cmake/configs/posix_sitl_simple.cmake)

list(APPEND config_module_list
	modules/local_position_estimator
	)

set(config_sitl_rcS
	posix-configs/SITL/init/rcS_lpe
	)
