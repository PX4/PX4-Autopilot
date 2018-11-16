include(default)

set(FW_NAME px4fmu-v3_rtps.elf CACHE string "" FORCE)

list(APPEND config_module_list
	modules/micrortps_bridge
)
