cmake_minimum_required(VERSION 3.11.0 FATAL_ERROR)

find_package(gz-cmake3 REQUIRED)

project(OpticalFlowSystem)

gz_find_package(gz-plugin2 REQUIRED COMPONENTS register)
set(GZ_PLUGIN_VER ${gz-plugin2_VERSION_MAJOR})

gz_find_package(gz-sim8 REQUIRED)
set(GZ_SIM_VER ${gz-sim8_VERSION_MAJOR})

find_package(gz-sensors8 REQUIRED)
set(GZ_SENSORS_VER ${gz-sensors8_VERSION_MAJOR})

find_package(OpenCV REQUIRED)

# add_subdirectory(sensors optical_flow)

PROTOBUF_GENERATE_CPP(PROTO_SRCS PROTO_HDRS optical_flow.proto)

add_library(optical_flow_plugin SHARED
  ${PROTO_SRCS}
  ${PROTO_HDRS}
  OpticalFlow.cc
)

set(sensors
  optical_flow_plugin
)

foreach(sensor ${sensors})
  target_link_libraries(${sensor}
    PUBLIC gz-sensors8::gz-sensors8
    PUBLIC gz-rendering8::gz-rendering8
    PUBLIC ${OpenCV_LIBS}
    PUBLIC ${PROTOBUF_LIBRARIES}
  )

  target_include_directories(${sensor}
    PUBLIC ${CMAKE_CURRENT_BINARY_DIR}
    PUBLIC ${OpenCV_INCLUDE_DIRS}
  )

endforeach()

add_library(${PROJECT_NAME} SHARED
  OpticalFlowSystem.cc
)

# add_dependencies(${PROJECT_NAME} optical_flow)

target_link_libraries(${PROJECT_NAME}
  PUBLIC ${sensors}
  PRIVATE gz-plugin${GZ_PLUGIN_VER}::gz-plugin${GZ_PLUGIN_VER}
  PRIVATE gz-sim${GZ_SIM_VER}::gz-sim${GZ_SIM_VER}
  PRIVATE gz-sensors${GZ_SENSORS_VER}::gz-sensors${GZ_SENSORS_VER}
  PRIVATE ${OpenCV_LIBS}
)

target_include_directories(${PROJECT_NAME}
    PUBLIC ${PROJECT_SOURCE_DIR}
    PUBLIC ${CMAKE_CURRENT_BINARY_DIR}
    PUBLIC ${OpenCV_INCLUDE_DIRS}
)
