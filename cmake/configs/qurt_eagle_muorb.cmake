include(configs/qcom/qurt_eagle_common)

set(config_module_list
	drivers/device

	#
	# System commands
	#
	systemcmds/param

	#
	# Library modules
	#
	modules/param
	modules/systemlib
	modules/uORB

	#
	# Libraries
	#
	lib/mathlib
	lib/mathlib/math/filter
	lib/geo
	lib/geo_lookup
	lib/conversion
	lib/version
	lib/DriverFramework/framework

	#
	# QuRT port
	#
	platforms/common
	platforms/qurt/px4_layer
	platforms/posix/work_queue
	platforms/qurt/tests/muorb

	#
	# sources for muorb over fastrpc
	#
	modules/muorb/adsp
	)
