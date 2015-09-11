include(nuttx/px4_impl_nuttx)

message(WARNING "this is a work in progress and doesn't build yet")

function(px4_get_config)

	px4_parse_function_args(
		NAME px4_set_config_modules
		ONE_VALUE OUT_MODULES OUT_FW_OPTS OUT_EXTRA_CMDS
		ARGN ${ARGN})

	set(config_module_list
		platforms/nuttx
		platforms/nuttx/px4_layer
		platforms/common
		drivers/led
		drivers/device
		modules/systemlib
		modules/uORB
		examples/px4_simple_app
		lib/mathlib/math/filter
		lib/conversion
		)

	set(${out_module_list} ${config_module_list} PARENT_SCOPE)

endfunction()

