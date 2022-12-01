cmake_minimum_required(VERSION 3.5)
project(px4_sdk_lib)

# Default to C++17
if(NOT CMAKE_CXX_STANDARD)
	set(CMAKE_CXX_STANDARD 17)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
	add_compile_options(-Wall -Wextra -Wpedantic -Werror -Wno-unused-parameter)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(px4_msgs REQUIRED)
find_package(eigen3_cmake_module REQUIRED)
find_package(Eigen3 REQUIRED)

include_directories(include ${Eigen3_INCLUDE_DIRS})
set(HEADER_FILES
		include/px4_sdk/components/events.h
		include/px4_sdk/components/health_and_arming_checks.h
		include/px4_sdk/components/mode.h
		include/px4_sdk/components/mode_executor.h
		include/px4_sdk/components/overrides.h
		include/px4_sdk/components/setpoints.h
		include/px4_sdk/components/wait_for_fmu.h
	)

add_library(px4_sdk_lib
		src/components/health_and_arming_checks.cpp
		src/components/mode.cpp
		src/components/mode_executor.cpp
		src/components/overrides.cpp
		src/components/registration.cpp
		src/components/setpoints.cpp
		src/components/wait_for_fmu.cpp
		${HEADER_FILES}
	)
ament_target_dependencies(px4_sdk_lib Eigen3 rclcpp px4_msgs)

ament_export_targets(px4_sdk_lib HAS_LIBRARY_TARGET)
ament_export_dependencies(
    px4_msgs
    rclcpp
	eigen3_cmake_module
	Eigen3
)
install(
	DIRECTORY include/px4_sdk
	DESTINATION include
)
install(
	TARGETS px4_sdk_lib
	EXPORT px4_sdk_lib
	LIBRARY DESTINATION lib
	ARCHIVE DESTINATION lib
	RUNTIME DESTINATION bin
	INCLUDES DESTINATION include
)

if(BUILD_TESTING)
	find_package(ament_lint_auto REQUIRED)
	find_package(ament_cmake_gtest REQUIRED)
	ament_lint_auto_find_test_dependencies()

	add_library(integration_utils
			test/integration/util.cpp
			)
	include_directories(include)
	ament_target_dependencies(integration_utils rclcpp px4_msgs)

	ament_add_gtest(test_arming_check test/integration/arming_check.cpp)
	if(TARGET test_arming_check)
		target_link_libraries(test_arming_check ${PROJECT_NAME} integration_utils)
		target_include_directories(test_arming_check PRIVATE include)
	endif()

	ament_add_gtest(test_mode test/integration/mode.cpp)
	if(TARGET test_mode)
		target_link_libraries(test_mode ${PROJECT_NAME} integration_utils)
		target_include_directories(test_mode PRIVATE include)
	endif()

	ament_add_gtest(test_mode_executor test/integration/mode_executor.cpp)
	if(TARGET test_mode_executor)
		target_link_libraries(test_mode_executor ${PROJECT_NAME} integration_utils)
		target_include_directories(test_mode_executor PRIVATE include)
	endif()

	ament_add_gtest(test_overrides test/integration/overrides.cpp)
	if(TARGET test_overrides)
		target_link_libraries(test_overrides ${PROJECT_NAME} integration_utils)
		target_include_directories(test_overrides PRIVATE include)
	endif()

endif()

ament_package()
