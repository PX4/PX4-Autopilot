############################################################################
#
#   Copyright (c) 2022 PX4 Development Team. All rights reserved.
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions
# are met:
#
# 1. Redistributions of source code must retain the above copyright
#    notice, this list of conditions and the following disclaimer.
# 2. Redistributions in binary form must reproduce the above copyright
#    notice, this list of conditions and the following disclaimer in
#    the documentation and/or other materials provided with the
#    distribution.
# 3. Neither the name PX4 nor the names of its contributors may be
#    used to endorse or promote products derived from this software
#    without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
# "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
# LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
# FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
# COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
# INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
# BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS
# OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
# AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
# LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
# ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
# POSSIBILITY OF SUCH DAMAGE.
#
############################################################################

#include_directories(${CMAKE_CURRENT_BINARY_DIR})

message(STATUS "ROS_VERSION: $ENV{ROS_VERSION}")
message(STATUS "AMENT_PREFIX_PATH: $ENV{AMENT_PREFIX_PATH}")

get_property(module_libraries GLOBAL PROPERTY PX4_MODULE_LIBRARIES)

set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/bin)
set(CMAKE_RUNTIME_OUTPUT_DIRECTORY_DEBUG ${CMAKE_BINARY_DIR}/bin)
set(CMAKE_RUNTIME_OUTPUT_DIRECTORY_RELEASE ${CMAKE_BINARY_DIR}/bin)

# add_executable(px4
# 	src/px4/common/main.cpp
# )

add_compile_options(
	-Wno-error
	-Wno-float-equal # msg generated equality operator
)

include_directories(${PX4_BINARY_DIR})

# relative path to msgs
set(ros_msg_files)
foreach(msg_file ${msg_files})
	file(RELATIVE_PATH relative_path ${CMAKE_CURRENT_SOURCE_DIR} ${msg_file})
	list(APPEND ros_msg_files ${relative_path})
endforeach()

rosidl_generate_interfaces(${PROJECT_NAME}
	${ros_msg_files}
)

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
