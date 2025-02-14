cmake_minimum_required(VERSION 3.11.0 FATAL_ERROR)
find_package(gz-cmake3 REQUIRED)

project(OpticalFlowSystem)

gz_find_package(gz-plugin2 REQUIRED COMPONENTS register)
gz_find_package(gz-sim8 REQUIRED)
gz_find_package(gz-sensors8 REQUIRED)
gz_find_package(gz-transport13 REQUIRED)
find_package(OpenCV REQUIRED)

include(ExternalProject)

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
    PUBLIC gz-sensors8::gz-sensors8
    PUBLIC gz-rendering8::gz-rendering8
    PUBLIC gz-plugin2::gz-plugin2
    PUBLIC gz-sim8::gz-sim8
    PUBLIC gz-transport13::gz-transport13
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
