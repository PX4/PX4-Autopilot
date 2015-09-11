include(posix/px4_impl_posix-arm)

function(px4_get_config out_module_list)

	set(config_module_list
		drivers/device

		modules/uORB

		platforms/posix/px4_layer
		platforms/posix/work_queue

		modules/muorb/krait
		)

	set(${out_module_list} ${config_module_list} PARENT_SCOPE)

endfunction()

