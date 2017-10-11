include(configs/nuttx_px4fmu-v4_default)

list(APPEND config_module_list
	modules/micrortps_bridge
)