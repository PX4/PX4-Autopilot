px4_add_module(
	MODULE drivers__hps167
	MAIN hps167
	SRCS
		HPS167.cpp
		HPS167.hpp
		hps167_main.cpp
	MODULE_CONFIG
		module.yaml
	DEPENDS
		drivers_rangefinder
	)
