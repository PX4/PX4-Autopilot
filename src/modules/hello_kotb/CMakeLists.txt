px4_add_module(
	MODULE modules__hello_kotb
	MAIN hello_kotb
	STACK_MAIN 2000
	SRCS
		hello_kotb.cpp
		hello_kotb_main.cpp
	DEPENDS
)
