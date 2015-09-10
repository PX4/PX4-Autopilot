include(posix/px4_impl_posix-arm)

function(px4_set_config_modules out_module_list)

	set(config_module_list
		drivers/device

		systemcmds/param
		systemcmds/ver

		modules/mavlink

		modules/systemlib
		modules/uORB
		modules/dataman

		lib/mathlib
		lib/mathlib/math/filter
		lib/geo
		lib/geo_lookup
		lib/conversion

		platforms/posix/px4_layer
		platforms/posix/work_queue

		modules/muorb/krait
		)

	set(${out_module_list} ${config_module_list} PARENT_SCOPE)

endfunction()

