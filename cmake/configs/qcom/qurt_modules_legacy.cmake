# Modules used for legacy (binary driver) QuRT build

include(configs/qcom/qurt_modules_common)

add_definitions(
   -D__USING_SNAPDRAGON_LEGACY_DRIVER
   )

set(config_module_list ${config_module_list}
	# FC_ADDON drivers
	#
	platforms/qurt/fc_addon/rc_receiver
	platforms/qurt/fc_addon/uart_esc
	)
