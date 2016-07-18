include_directories(..)

add_executable(navio_gpio_test
	main.cpp
	test.cpp
	)

target_link_libraries(navio_gpio_test
	drivers__navio_gpio
	platforms__posix__px4_layer
	df_driver_framework
	)
