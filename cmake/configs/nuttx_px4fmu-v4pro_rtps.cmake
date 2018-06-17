include(configs/nuttx_px4fmu-v4pro_default)

list(APPEND config_module_list
	modules/micrortps_bridge
)