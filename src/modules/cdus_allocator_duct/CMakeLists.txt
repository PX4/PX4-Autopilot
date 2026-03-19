px4_add_module(
	MODULE modules__cdus_allocator_duct
	MAIN cdus_allocator_duct
	COMPILE_FLAGS
		${MAX_CUSTOM_OPT_LEVEL}
	SRCS
		CdusAllocatorDuct.cpp
		CdusAllocatorDuct.hpp
	DEPENDS
		px4_work_queue
	)
