# Modules used for default QuRT build

include(configs/qcom/qurt_modules_common)

set(config_module_list ${config_module_list}
	#
	# Board support modules
	#
	platforms/posix/drivers/df_hmc5883_wrapper
	platforms/posix/drivers/df_trone_wrapper
	platforms/posix/drivers/df_isl29501_wrapper

	#
	# PX4 drivers
	#
	drivers/spektrum_rc

	#
	# Libraries
	#
	lib/rc
	)

set(config_df_driver_list ${config_df_driver_list}
	hmc5883
	trone
	isl29501
	)
