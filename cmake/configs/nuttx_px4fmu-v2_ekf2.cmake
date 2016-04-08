include(cmake/configs/nuttx_px4fmu-v2_base.cmake)

#list(REMOVE_ITEM config_module_list
	#)

list(APPEND config_module_list
	modules/ekf2
	)
