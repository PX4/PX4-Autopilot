px4_add_library(VoliroHoverTrajectoryCore VoliroHoverTrajectory.cpp VoliroHoverTrajectory.hpp)
target_link_libraries(VoliroHoverTrajectoryCore PRIVATE mathlib)
target_include_directories(VoliroHoverTrajectoryCore PUBLIC ${CMAKE_CURRENT_SOURCE_DIR})
px4_add_unit_gtest(SRC VoliroHoverTrajectoryTest.cpp LINKLIBS VoliroHoverTrajectoryCore)

px4_add_module(
	MODULE modules__voliro_hover
	MAIN voliro_hover
	STACK_MAIN 3000
	SRCS
		VoliroHover.cpp
		VoliroHover.hpp
	MODULE_CONFIG module.yaml
	DEPENDS
		mathlib
		VoliroHoverTrajectoryCore
		px4_work_queue
)
