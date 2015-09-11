include(qurt/px4_impl_qurt)

function(px4_get_config)

	px4_parse_function_args(
		NAME px4_set_config_modules
		ONE_VALUE OUT_MODULES
		REQUIRED
		ARGN ${ARGN})

	set(config_module_list
		drivers/device

		#
		# System commands
		#
		systemcmds/param

		#
		# Library modules
		#
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
	set(${OUT_MODULES} ${config_module_list} PARENT_SCOPE)

endfunction()

