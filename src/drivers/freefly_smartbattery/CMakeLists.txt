px4_add_module(
	MODULE drivers__freefly_smartbattery
	MAIN ff_smartbattery
	COMPILE_FLAGS
	SRCS
		ff_smartbattery.cpp
	DEPENDS
	)
