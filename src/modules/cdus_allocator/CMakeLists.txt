px4_add_module(
	MODULE modules__cdus_allocator
	MAIN cdus_allocator
	COMPILE_FLAGS
		${MAX_CUSTOM_OPT_LEVEL}
	SRCS
		CdusAllocator.cpp
		CdusAllocator.hpp
	DEPENDS
		px4_work_queue
	)
