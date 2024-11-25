px4_add_module(
	MODULE drivers__magnetometer__memsic__mmc5616
	MAIN mmc5616
	COMPILE_FLAGS
	SRCS
		MEMSIC_mmc5616_registers.hpp
		mmc5616.cpp
		mmc5616_i2c.cpp
		mmc5616.hpp
		mmc5616_main.cpp
	DEPENDS
		drivers_magnetometer
		px4_work_queue
	)
