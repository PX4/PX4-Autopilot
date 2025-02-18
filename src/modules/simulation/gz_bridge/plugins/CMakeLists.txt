cmake_minimum_required(VERSION 3.11.0 FATAL_ERROR)

project(OpticalFlowSystem)

gz_find_package(gz-plugin${GZ_PLUGIN_VERSION} REQUIRED COMPONENTS register)
gz_find_package(gz-sim${GZ_SIM_VERSION} REQUIRED)
gz_find_package(gz-sensors${GZ_SENSORS_VERSION} REQUIRED)

# PX4-OpticalFlow
include(ExternalProject)
find_package(OpenCV REQUIRED)
ExternalProject_Add(OpticalFlow
    GIT_REPOSITORY https://github.com/PX4/PX4-OpticalFlow.git
    GIT_TAG master
    PREFIX ${CMAKE_BINARY_DIR}/OpticalFlow
    INSTALL_DIR ${CMAKE_BINARY_DIR}/OpticalFlow/install
    CMAKE_ARGS -DCMAKE_INSTALL_PREFIX=<INSTALL_DIR>
    BUILD_BYPRODUCTS ${CMAKE_BINARY_DIR}/OpticalFlow/install/lib/libOpticalFlow.so
)

ExternalProject_Get_Property(OpticalFlow install_dir)

set(OpticalFlow_INCLUDE_DIRS ${install_dir}/include)
set(OpticalFlow_LIBS ${install_dir}/lib/libOpticalFlow.so)

add_library(${PROJECT_NAME} SHARED
    OpticalFlowSensor.cpp
    OpticalFlowSystem.cpp
)

target_link_libraries(${PROJECT_NAME}
    PUBLIC px4_msgs
    PUBLIC gz-sensors${GZ_SENSORS_VERSION}::gz-sensors${GZ_SENSORS_VERSION}
    # PUBLIC gz-rendering8::gz-rendering8
    PUBLIC gz-plugin${GZ_PLUGIN_VERSION}::gz-plugin${GZ_PLUGIN_VERSION}
    PUBLIC gz-sim${GZ_SIM_VERSION}::gz-sim${GZ_SIM_VERSION}
    PUBLIC gz-transport${GZ_TRANSPORT_VERSION}::gz-transport${GZ_TRANSPORT_VERSION}
    PUBLIC ${OpenCV_LIBS}
    PUBLIC ${OpticalFlow_LIBS}
)

target_include_directories(${PROJECT_NAME}
    PUBLIC ${CMAKE_CURRENT_SOURCE_DIR}
    PUBLIC ${CMAKE_CURRENT_BINARY_DIR}
    PUBLIC ${OpenCV_INCLUDE_DIRS}
    PUBLIC ${OpticalFlow_INCLUDE_DIRS}
)

add_dependencies(${PROJECT_NAME} OpticalFlow)
