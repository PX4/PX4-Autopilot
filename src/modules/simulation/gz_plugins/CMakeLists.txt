project(OpticalFlowSystem)

if(NOT DEFINED ENV{GZ_DISTRO})
    set(GZ_DISTRO "harmonic" CACHE STRING "Gazebo distribution to use")
else()
    set(GZ_DISTRO $ENV{GZ_DISTRO})
endif()

# Define library version combinations for different Gazebo distributions
# https://github.com/gazebo-tooling/gazebodistro/blob/master/collection-harmonic.yaml
if(GZ_DISTRO STREQUAL "harmonic")
    set(GZ_CMAKE_VERSION "3")
    set(GZ_MSGS_VERSION "10")
    set(GZ_TRANSPORT_VERSION "13")
    set(GZ_PLUGIN_VERSION "2")
    set(GZ_SIM_VERSION "8")
    set(GZ_SENSORS_VERSION "8")
    message(STATUS "Using Gazebo Harmonic (cmake:${GZ_CMAKE_VERSION}, msgs:${GZ_MSGS_VERSION}, transport:${GZ_TRANSPORT_VERSION})")
elseif(GZ_DISTRO STREQUAL "ionic")
    set(GZ_CMAKE_VERSION "4")
    set(GZ_MSGS_VERSION "11")
    set(GZ_TRANSPORT_VERSION "14")
    set(GZ_PLUGIN_VERSION "3")
    set(GZ_SIM_VERSION "9")
    set(GZ_SENSORS_VERSION "9")
    message(STATUS "Using Gazebo Ionic (cmake:${GZ_CMAKE_VERSION}, msgs:${GZ_MSGS_VERSION}, transport:${GZ_TRANSPORT_VERSION})")
else()
    message(FATAL_ERROR "Unknown Gazebo distribution: ${GZ_DISTRO}. Valid options are: harmonic or ionic")
endif()

# Use gz-transport as litmus test for prescence of gz
find_package(gz-transport${GZ_TRANSPORT_VERSION})

if (gz-transport${GZ_TRANSPORT_VERSION}_FOUND)

    gz_find_package(gz-cmake${GZ_CMAKE_VERSION} REQUIRED)
    gz_find_package(gz-msgs${GZ_MSGS_VERSION} REQUIRED)
    gz_find_package(Protobuf REQUIRED)
    gz_find_package(gz-plugin${GZ_PLUGIN_VERSION} REQUIRED COMPONENTS register)
    gz_find_package(gz-sim${GZ_SIM_VERSION} REQUIRED)
    gz_find_package(gz-sensors${GZ_SENSORS_VERSION} REQUIRED)

    include(${CMAKE_CURRENT_SOURCE_DIR}/optical_flow.cmake)

    add_library(${PROJECT_NAME} SHARED
        OpticalFlowSensor.cpp
        OpticalFlowSystem.cpp
    )

    target_link_libraries(${PROJECT_NAME}
        PUBLIC px4_gz_msgs
        PUBLIC gz-sensors${GZ_SENSORS_VERSION}::gz-sensors${GZ_SENSORS_VERSION}
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
        PUBLIC px4_gz_msgs
    )

    add_dependencies(${PROJECT_NAME} OpticalFlow)

endif()
