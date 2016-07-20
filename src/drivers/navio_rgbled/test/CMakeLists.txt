include_directories(..)

add_executable(navio_rgbled_test
	main.cpp
	test.cpp
	)

target_link_libraries(navio_rgbled_test
	drivers__navio_rgbled
	drivers__navio_gpio
	platforms__posix__px4_layer
	df_driver_framework
	)
