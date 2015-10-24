include(cmake/configs/posix_sitl_default.cmake)

list(APPEND config_module_list
	modules/local_position_estimator
	)

set(config_sitl_rcS
	posix-configs/SITL/init/rcS_lpe
	)
