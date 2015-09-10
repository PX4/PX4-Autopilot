include(nuttx/px4_impl_nuttx)

function(px4_set_config_modules out_module_list)

	set(config_module_list
		platforms/nuttx
		platforms/nuttx/px4_layer
		platforms/common
		drivers/led
		drivers/device
		modules/systemlib
		modules/uORB
		examples/px4_simple_app
		drivers/boards/px4fmu-v2
		drivers/stm32
		lib/mathlib/math/filter
		lib/conversion
		)

	set(${out_module_list} ${config_module_list} PARENT_SCOPE)

endfunction()

