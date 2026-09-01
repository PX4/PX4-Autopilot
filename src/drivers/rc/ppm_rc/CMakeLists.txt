px4_add_module(
	MODULE drivers__rc__ppm_rc
	MAIN ppm_rc
	SRCS
		PpmRc.cpp
		PpmRc.hpp
	MODULE_CONFIG
		module.yaml
	)
