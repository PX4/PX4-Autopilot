include(cmake/configs/nuttx_px4fmu-v2_base.cmake)

#list(REMOVE_ITEM config_module_list
	#)

list(APPEND config_module_list
	modules/attitude_estimator_q
	modules/position_estimator_inav
	)
