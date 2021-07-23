add_library(drivers_board
	i2c.cpp
	init.c
	led.c
	# spi.cpp
	# timer_config.cpp
	# usb.c
)

target_link_libraries(drivers_board
	PRIVATE
		# arch_spi
		# drivers__led # drv_led_start
		nuttx_arch # sdio
		nuttx_drivers # sdio
		px4_layer
)
