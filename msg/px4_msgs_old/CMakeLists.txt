cmake_minimum_required(VERSION 3.5)

project(px4_msgs_old)

list(INSERT CMAKE_MODULE_PATH 0 "${CMAKE_CURRENT_SOURCE_DIR}/cmake")

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
	add_compile_options(-Wall -Wextra)
endif()

find_package(ament_cmake REQUIRED)
find_package(builtin_interfaces REQUIRED)
find_package(rosidl_default_generators REQUIRED)

# ##############################################################################
# Generate ROS messages, ROS2 interfaces and IDL files #
# ##############################################################################

# get all msg files
set(MSGS_DIR "${CMAKE_CURRENT_SOURCE_DIR}/msg")
file(GLOB PX4_MSGS RELATIVE "${CMAKE_CURRENT_SOURCE_DIR}" "${MSGS_DIR}/*.msg")

# get all srv files
set(SRVS_DIR "${CMAKE_CURRENT_SOURCE_DIR}/srv")
file(GLOB PX4_SRVS RELATIVE "${CMAKE_CURRENT_SOURCE_DIR}" "${SRVS_DIR}/*.srv")



# For the versioned topics, replace the namespace (px4_msgs_old -> px4_msgs) and message type name (<msg>Vx -> <msg>),
# so that DDS does not reject the subscription/publication due to mismatching type
# rosidl_typesupport_fastrtps_cpp
set(rosidl_typesupport_fastrtps_cpp_BIN ${CMAKE_CURRENT_BINARY_DIR}/rosidl_typesupport_fastrtps_cpp_wrapper.py)
file(TOUCH ${rosidl_typesupport_fastrtps_cpp_BIN})

# rosidl_typesupport_fastrtps_c
set(rosidl_typesupport_fastrtps_c_BIN ${CMAKE_CURRENT_BINARY_DIR}/rosidl_typesupport_fastrtps_c_wrapper.py)
file(TOUCH ${rosidl_typesupport_fastrtps_c_BIN})

# rosidl_typesupport_introspection_cpp (for cyclonedds)
set(rosidl_typesupport_introspection_cpp_BIN ${CMAKE_CURRENT_BINARY_DIR}/rosidl_typesupport_introspection_cpp_wrapper.py)
file(TOUCH ${rosidl_typesupport_introspection_cpp_BIN})

# Generate introspection typesupport for C and C++ and IDL files
if(PX4_MSGS)
	rosidl_generate_interfaces(${PROJECT_NAME}
			${PX4_MSGS}
			${PX4_SRVS}
			DEPENDENCIES builtin_interfaces
			ADD_LINTER_TESTS
	)
endif()

# rosidl_typesupport_fastrtps_cpp
set(rosidl_typesupport_fastrtps_cpp_orig ${rosidl_typesupport_fastrtps_cpp_DIR})
string(REPLACE "share/rosidl_typesupport_fastrtps_cpp/cmake" "lib/rosidl_typesupport_fastrtps_cpp/rosidl_typesupport_fastrtps_cpp"
		rosidl_typesupport_fastrtps_cpp_orig ${rosidl_typesupport_fastrtps_cpp_DIR})
set(original_script_path ${rosidl_typesupport_fastrtps_cpp_orig})
configure_file(rename_msg_type.py.in ${rosidl_typesupport_fastrtps_cpp_BIN} @ONLY)

# rosidl_typesupport_fastrtps_c
set(rosidl_typesupport_fastrtps_c_orig ${rosidl_typesupport_fastrtps_c_DIR})
string(REPLACE "share/rosidl_typesupport_fastrtps_c/cmake" "lib/rosidl_typesupport_fastrtps_c/rosidl_typesupport_fastrtps_c"
		rosidl_typesupport_fastrtps_c_orig ${rosidl_typesupport_fastrtps_c_DIR})
set(original_script_path ${rosidl_typesupport_fastrtps_c_orig})
configure_file(rename_msg_type.py.in ${rosidl_typesupport_fastrtps_c_BIN} @ONLY)

# rosidl_typesupport_introspection_cpp
set(rosidl_typesupport_introspection_cpp_orig ${rosidl_typesupport_introspection_cpp_DIR})
string(REPLACE "share/rosidl_typesupport_introspection_cpp/cmake" "lib/rosidl_typesupport_introspection_cpp/rosidl_typesupport_introspection_cpp"
		rosidl_typesupport_introspection_cpp_orig ${rosidl_typesupport_introspection_cpp_DIR})
set(original_script_path ${rosidl_typesupport_introspection_cpp_orig})
configure_file(rename_msg_type.py.in ${rosidl_typesupport_introspection_cpp_BIN} @ONLY)

ament_export_dependencies(rosidl_default_runtime)

ament_package()
