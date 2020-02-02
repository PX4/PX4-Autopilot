px4_add_module(

	MODULE drivers__pcf8583
	MAIN pcf8583
	COMPILE_FLAGS
	SRCS
		pcf8583.cpp
	DEPENDS
      drivers__device
)
