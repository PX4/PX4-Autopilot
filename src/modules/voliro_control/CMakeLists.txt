px4_add_library(VoliroControlCore VoliroControl.cpp VoliroControl.hpp)
target_link_libraries(VoliroControlCore PRIVATE mathlib)
target_include_directories(VoliroControlCore PUBLIC ${CMAKE_CURRENT_SOURCE_DIR})
px4_add_unit_gtest(SRC VoliroControlTest.cpp LINKLIBS VoliroControlCore)

px4_add_module(
	MODULE modules__voliro_control
	MAIN voliro_control
	STACK_MAIN 2500
	SRCS VoliroControlModule.cpp VoliroControlModule.hpp
	MODULE_CONFIG module.yaml
	DEPENDS mathlib VoliroControlCore px4_work_queue
)
