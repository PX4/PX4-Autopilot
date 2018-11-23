px4_add_module(
	MODULE platforms__posix__tests__dyn_hello
	MAIN hello
	SRCS
		hello.cpp
	DEPENDS
	DYNAMIC
	)
