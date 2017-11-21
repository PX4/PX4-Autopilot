include(configs/nuttx_aerofc-v1_default)


list(APPEND config_module_list
	modules/micrortps_bridge
	drivers/protocol_splitter
)
