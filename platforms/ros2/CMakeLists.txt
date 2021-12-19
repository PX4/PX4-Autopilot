
#include_directories(${CMAKE_CURRENT_BINARY_DIR})

get_property(module_libraries GLOBAL PROPERTY PX4_MODULE_LIBRARIES)

# When building with catkin, do not change CMAKE_RUNTIME_OUTPUT_DIR from
# CMAKE_CURRENT_BINARY_DIR ''./platforms/posix' to './bin'
if (NOT CATKIN_DEVEL_PREFIX)
	set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/bin)
	set(CMAKE_RUNTIME_OUTPUT_DIRECTORY_DEBUG ${CMAKE_BINARY_DIR}/bin)
	set(CMAKE_RUNTIME_OUTPUT_DIRECTORY_RELEASE ${CMAKE_BINARY_DIR}/bin)
endif()


# add_executable(px4
# 	src/px4/common/main.cpp
# )

add_compile_options(-Wno-error)

include_directories(${PX4_BINARY_DIR})

# # mc_rate_control_ros2
# add_executable(mc_rate_control ${CMAKE_SOURCE_DIR}/src/modules/mc_rate_control_ros2/MulticopterRateControl.cpp)
# target_link_libraries(mc_rate_control px4_layer work_queue)
# ament_target_dependencies(mc_rate_control rclcpp std_msgs)
# rosidl_target_interfaces(mc_rate_control ${PROJECT_NAME} "rosidl_typesupport_cpp")
# add_dependencies(mc_rate_control generate_parameters)

# install(
#   TARGETS
#     mc_rate_control
#   DESTINATION lib/${PROJECT_NAME}
# )
