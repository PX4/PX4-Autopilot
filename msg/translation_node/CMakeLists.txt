cmake_minimum_required(VERSION 3.8)
project(translation_node)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic -Wno-unused-parameter -Werror)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(px4_msgs REQUIRED)
find_package(px4_msgs_old REQUIRED)

if(DEFINED ENV{ROS_DISTRO})
  set(ROS_DISTRO $ENV{ROS_DISTRO})
else()
  set(ROS_DISTRO "rolling")
endif()


add_library(${PROJECT_NAME}_lib
        src/monitor.cpp
        src/pub_sub_graph.cpp
        src/service_graph.cpp
        src/translations.cpp
)
ament_target_dependencies(${PROJECT_NAME}_lib rclcpp px4_msgs px4_msgs_old)
add_executable(${PROJECT_NAME}_bin
        src/main.cpp
)
target_link_libraries(${PROJECT_NAME}_bin ${PROJECT_NAME}_lib)
target_include_directories(${PROJECT_NAME}_bin PUBLIC src)
ament_target_dependencies(${PROJECT_NAME}_bin rclcpp px4_msgs px4_msgs_old)
install(TARGETS
        ${PROJECT_NAME}_bin
        DESTINATION lib/${PROJECT_NAME})

option(DISABLE_SERVICES "Disable services" OFF)
if(${ROS_DISTRO} STREQUAL "humble")
  message(WARNING "Disabling services for ROS humble (API is not supported)")
  target_compile_definitions(${PROJECT_NAME}_lib PRIVATE DISABLE_SERVICES)
  set(DISABLE_SERVICES ON)
endif()

if(BUILD_TESTING)
  find_package(std_msgs REQUIRED)
  find_package(ament_lint_auto REQUIRED)
  find_package(ament_cmake_gtest REQUIRED)
  find_package(rosidl_default_generators REQUIRED)
  ament_lint_auto_find_test_dependencies()

  set(SRV_FILES
          test/srv/TestV0.srv
          test/srv/TestV1.srv
          test/srv/TestV2.srv
  )
  rosidl_generate_interfaces(${PROJECT_NAME} ${SRV_FILES})

  # Unit tests
  set(TEST_SRC
          test/graph.cpp
          test/main.cpp
          test/pub_sub.cpp
  )
  if (NOT DISABLE_SERVICES)
    list(APPEND TEST_SRC test/services.cpp)
  endif()
  ament_add_gtest(${PROJECT_NAME}_unit_tests
          ${TEST_SRC}
  )
  target_include_directories(${PROJECT_NAME}_unit_tests PRIVATE ${CMAKE_CURRENT_LIST_DIR})
  target_compile_options(${PROJECT_NAME}_unit_tests PRIVATE -Wno-error=sign-compare) # There is a warning from gtest internal
  target_link_libraries(${PROJECT_NAME}_unit_tests ${PROJECT_NAME}_lib)
  rosidl_get_typesupport_target(cpp_typesupport_target ${PROJECT_NAME} "rosidl_typesupport_cpp")
  target_link_libraries(${PROJECT_NAME}_unit_tests "${cpp_typesupport_target}")
  ament_target_dependencies(${PROJECT_NAME}_unit_tests
          std_msgs
          rclcpp
  )
endif()

ament_package()
