include(configs/qcom/posix_eagle_common)

set(config_module_list
	drivers/device

	modules/uORB

	lib/DriverFramework/framework

	platforms/posix/px4_layer
	platforms/posix/work_queue

	modules/muorb/krait
	)

