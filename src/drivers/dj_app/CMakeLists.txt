px4_add_module(
	MODULE drivers__dj_app
	MAIN dj_app
	SRCS
		dj_app.cpp
	DEPENDS
		mixer_module
	MODULE_CONFIG
		module.yaml
	)
