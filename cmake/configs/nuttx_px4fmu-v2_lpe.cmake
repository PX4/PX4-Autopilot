include(cmake/configs/nuttx_px4fmu-v2_base.cmake)

list(REMOVE_ITEM config_module_list
	modules/ekf_att_pos_estimator
	)

list(APPEND config_module_list
	modules/local_position_estimator
	modules/attitude_estimator_q
	)
