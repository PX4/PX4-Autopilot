px4_add_library(VoliroAllocation VoliroAllocation.cpp VoliroAllocation.hpp)
target_link_libraries(VoliroAllocation PRIVATE mathlib)
target_include_directories(VoliroAllocation PUBLIC ${CMAKE_CURRENT_SOURCE_DIR})
px4_add_unit_gtest(SRC VoliroAllocationTest.cpp LINKLIBS VoliroAllocation)

px4_add_module(
	MODULE modules__voliro_allocator
	MAIN voliro_allocator
	STACK_MAIN 2500
	SRCS VoliroAllocator.cpp VoliroAllocator.hpp
	MODULE_CONFIG module.yaml
	DEPENDS mathlib VoliroAllocation px4_work_queue
)
